import hashlib
import ipaddress
import shlex
import subprocess
import os
import platform
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field, validator

app = FastAPI(title="VXLAN REST Configurator", version="1.0.0")


# ---------- Helpers ----------
def _probe(cmd: str) -> tuple[bool, str]:
    """Non-raising probe used by /health."""
    try:
        p = subprocess.run(cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        return (p.returncode == 0, (p.stdout or "").strip())
    except Exception as e:
        return (False, str(e))
    
def run(cmd: str) -> str:
    """Run a shell command safely and return stdout; raise on non-zero."""
    try:
        out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT, text=True)
        return out.strip()
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"{cmd} -> {e.output.strip()}")

def link_exists(dev: str) -> bool:
    try:
        run(f"ip link show {dev}")
        return True
    except HTTPException:
        return False

def strip_cidr(ip: str) -> str:
    return str(ipaddress.ip_interface(ip).ip)

def mac_from_seed(seed: str) -> str:
    """Deterministic locally-administered unicast MAC from seed."""
    h = hashlib.sha256(seed.encode()).hexdigest()
    # set locally-administered (x2) and unicast (x0) on first byte
    b0 = int(h[0:2], 16) & 0b11111100 | 0b00000010
    rest = [h[i:i+2] for i in range(2, 12, 2)]
    return ":".join([f"{b0:02x}", *rest])

def is_ipv4(ip: str) -> bool:
    try:
        ipaddress.IPv4Address(ip)
        return True
    except Exception:
        return False

def ensure_root():
    # Best-effort check. You can also run uvicorn with sudo/systemd.
    import os
    if os.geteuid() != 0:
        raise HTTPException(status_code=403, detail="Root privileges required.")


# ---------- Schemas ----------
class CreateVxlanReq(BaseModel):
    vni: int = Field(ge=1, le=16777215, description="VXLAN Network Identifier")
    iface: str = Field(min_length=1, description="Underlay NIC (e.g., ens3)")
    port: int = Field(default=4789, ge=1, le=65535)
    vxlan_ip: str = Field(description="VXLAN interface IP (CIDR ok, e.g., 172.20.50.10/24)")
    remote_ips: List[str] = Field(default_factory=list)
    local_ip: Optional[str] = Field(default=None, description="Bind source IP (optional)")
    force: bool = False

    @validator("remote_ips", each_item=True)
    def _check_peers(cls, v):
        if not is_ipv4(v):
            raise ValueError(f"Invalid IPv4: {v}")
        return v

    @validator("vxlan_ip")
    def _check_vxlan_ip(cls, v):
        # validates CIDR or plain IP
        try:
            ipaddress.ip_interface(v)
            return v
        except Exception:
            raise ValueError("vxlan_ip must be IPv4 with/without prefix, e.g., 172.20.50.10/24")


class AddPeersReq(BaseModel):
    peers: List[str]

    @validator("peers", each_item=True)
    def _check_ip(cls, v):
        if not is_ipv4(v):
            raise ValueError(f"Invalid IPv4: {v}")
        return v


# ---------- Endpoints ----------
@app.get("/health", summary="Health check / readiness probe")
def health():
    # Basic environment info
    py = platform.python_version()
    uid = os.geteuid() if hasattr(os, "geteuid") else None

    # Tooling present?
    ip_ok, ip_out = _probe("ip -V")
    br_ok, br_out = _probe("bridge -V")

    # VXLAN kernel signal (best effort, non-root friendly)
    vx_ok, vx_out = _probe("lsmod")
    vxlan_loaded = vx_ok and any(line.startswith("vxlan ") for line in vx_out.splitlines())

    # Existing vxlan links (read-only)
    vx_links_ok, vx_links_out = _probe("ip -d link show type vxlan")
    vx_links = []
    if vx_links_ok:
        # Extract device names from lines like: "5: vxlan200: <BROADCAST,MULTICAST,UP,LOWER_UP> ..."
        for line in vx_links_out.splitlines():
            parts = line.split(": ", 2)
            if len(parts) >= 2 and parts[1].startswith("vxlan"):
                vx_links.append(parts[1].split(":")[0])

    status = "ok" if (ip_ok and br_ok) else "degraded"

    return {
        "status": status,
        "python": py,
        "process_uid": uid,
        "binaries": {
            "ip": ip_ok,
            "bridge": br_ok
        },
        "vxlan": {
            "kernel_module_loaded": vxlan_loaded,
            "links": vx_links,
        }
    }

@app.post("/vxlan", summary="Create VXLAN interface and add peers")
def create_vxlan(req: CreateVxlanReq):
    ensure_root()

    vx = f"vxlan{req.vni}"
    if link_exists(vx):
        if not req.force:
            raise HTTPException(status_code=409, detail=f"Interface {vx} already exists. Use force=true to recreate.")
        # best-effort down & delete
        run(f"ip link set {vx} down")
        run(f"ip link del {vx}")

    # Create VXLAN link
    base = f"ip link add {vx} type vxlan id {req.vni} dstport {req.port} dev {req.iface}"
    if req.local_ip:
        if not is_ipv4(req.local_ip):
            raise HTTPException(status_code=422, detail="local_ip must be IPv4")
        base += f" local {req.local_ip}"
    run(base)

    # Add FDB flood entries
    added = []
    for peer in req.remote_ips:
        try:
            run(f"bridge fdb append to 00:00:00:00:00:00 dev {vx} dst {peer}")
            added.append(peer)
        except HTTPException as e:
            # continue but record failure
            added.append(f"{peer} (failed: {e.detail})")

    # Assign IP
    run(f"ip addr add {req.vxlan_ip} dev {vx}")

    # Up and set MAC (deterministic from IP+VNI)
    vx_ip_nopfx = strip_cidr(req.vxlan_ip)
    mac = mac_from_seed(f"{vx_ip_nopfx}-{req.vni}")
    run(f"ip link set {vx} up")
    run(f"ip link set {vx} address {mac}")

    return {
        "iface": vx,
        "vni": req.vni,
        "port": req.port,
        "underlay_iface": req.iface,
        "vxlan_ip": req.vxlan_ip,
        "mac": mac,
        "peers_added": added,
        "message": "VXLAN setup complete",
    }


@app.post("/vxlan/{iface}/peers", summary="Add VXLAN peers (idempotent)")
def add_peers(iface: str, body: AddPeersReq):
    ensure_root()
    if not link_exists(iface):
        raise HTTPException(status_code=404, detail=f"{iface} not found")

    # Get current FDB once
    fdb = run(f"bridge fdb show dev {iface}").splitlines()
    def has_flood_entry(ip: str) -> bool:
        for line in fdb:
            # look for flood MAC with same dst
            if line.strip().startswith("00:00:00:00:00:00") and f" dst {ip}" in line:
                return True
        return False

    added, skipped, failed = [], [], []
    for p in body.peers:
        if has_flood_entry(p):
            skipped.append(p)
            continue
        try:
            run(f"bridge fdb append to 00:00:00:00:00:00 dev {iface} dst {p}")
            added.append(p)
        except HTTPException as e:
            failed.append({p: e.detail})

    return {
        "iface": iface,
        "added": added,
        "skipped": skipped,
        "failed": failed,
        "fdb": run(f"bridge fdb show dev {iface}").splitlines()
    }


@app.get("/vxlan/{iface}/fdb", summary="Show FDB entries for interface")
def show_fdb(iface: str):
    ensure_root()
    if not link_exists(iface):
        raise HTTPException(status_code=404, detail=f"{iface} not found")
    return {"iface": iface, "fdb": run(f"bridge fdb show dev {iface}").splitlines()}


@app.delete("/vxlan/{name}", summary="Delete VXLAN interface")
def delete_vxlan(name: str):
    ensure_root()
    if not link_exists(name):
        return {"message": f"{name} not found; nothing to do."}
    # detach from bridge if enslaved (best-effort)
    try:
        run(f"ip link set {name} nomaster")
    except HTTPException:
        pass
    try:
        run(f"ip link set {name} down")
    except HTTPException:
        pass
    run(f"ip link del {name}")
    return {"message": f"{name} removed."}
