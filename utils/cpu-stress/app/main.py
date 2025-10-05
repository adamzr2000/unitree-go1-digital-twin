from fastapi import FastAPI, Query
from fastapi.responses import JSONResponse
from multiprocessing import Process, Event, cpu_count
import time
from threading import Timer

app = FastAPI(title="Mini CPU Stresser")

# Global state for stress workers
workers: list[Process] = []
stop_events: list[Event] = []

def burn(stop_evt: Event):
    # Busy-loop until told to stop
    x = 0.0
    while not stop_evt.is_set():
        x = x * x + 1.234567  # pointless math to keep the core busy

def _terminate_all():
    # Signal and join all workers
    for ev in stop_events:
        ev.set()
    for p in workers:
        if p.is_alive():
            p.join(timeout=2)
    # clean leftovers
    for p in workers:
        if p.is_alive():
            p.terminate()
    workers.clear()
    stop_events.clear()

@app.get("/health")
def health():
    return {"status": "ok"}

@app.get("/status")
def status():
    alive = sum(1 for p in workers if p.is_alive())
    return {
        "workers_requested": len(workers),
        "workers_alive": alive,
        "max_cpus": cpu_count()
    }

@app.post("/stress")
def start_stress(
    n: int = Query(default=None, ge=1, description="Number of worker processes"),
    duration: float | None = Query(default=None, ge=0, description="Auto-stop after N seconds (optional)")
):
    """
    Start CPU stress with N workers (defaults to CPU count).
    Optionally auto-stop after `duration` seconds.
    """
    if n is None:
        n = cpu_count()
    n = min(n, cpu_count())

    # If already running, stop first (simple behavior)
    if any(p.is_alive() for p in workers):
        _terminate_all()

    for _ in range(n):
        ev = Event()
        p = Process(target=burn, args=(ev,), daemon=True)
        stop_events.append(ev)
        workers.append(p)
        p.start()

    if duration and duration > 0:
        # Use a thread-based timer — safe with our globals
        Timer(duration, _terminate_all).start()

    return JSONResponse({"message": "CPU stress started", "workers": n, "auto_stop_seconds": duration})

@app.post("/stop")
def stop_stress():
    _terminate_all()
    return {"message": "CPU stress stopped"}

@app.on_event("shutdown")
def on_shutdown():
    _terminate_all()
