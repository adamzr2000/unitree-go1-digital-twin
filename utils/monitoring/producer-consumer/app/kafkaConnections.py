# Copyright 2021 Scuola Superiore Sant'Anna www.santannapisa.it
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import logging

from confluent_kafka.admin import AdminClient, NewTopic
from confluent_kafka import Consumer, Producer
from http.client import HTTPConnection

log = logging.getLogger("KafkaConnections")


class kafkaConnections:
    def __init__(self, configfile: str = None):
        # defaults
        self.kIp = "10.30.2.35"
        self.kPort = 9291
        self.ktopic = "infrastructure"
        self.kID = 30

        # load JSON config
        cfg_path = configfile or 'default.json'
        try:
            with open(cfg_path, 'r') as f:
                cfg = json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            log.warning(f"Could not read/parse {cfg_path}, using defaults: {e}")
            cfg = {}

        kafka_cfg = cfg.get('kafka', {})
        self.kIp = kafka_cfg.get('kafkaIP', self.kIp)
        self.kPort = int(kafka_cfg.get('kafkaPort', self.kPort))
        self.ktopic = kafka_cfg.get('kafkaTopic', self.ktopic)
        if 'kafkaID' in kafka_cfg:
            self.kID = int(kafka_cfg['kafkaID'])


    ### KAFKA methods ###

    def createKafkaTopic(self, topic: str):
        broker = f"{self.kIp}:{self.kPort}"
        client = AdminClient({'bootstrap.servers': broker})
        new_topic = NewTopic(topic, num_partitions=1, replication_factor=1)
        fs = client.create_topics([new_topic])

        log.debug(f'External Connector: Creating kafka topic {topic}')
        for t, f in fs.items():
            try:
                f.result()
                log.debug(f"External Connector: Topic {t} created")
                return topic
            except Exception as e:
                log.error(f"External Connector: Failed to create topic {t}: {e}")
                return 0

    def deleteKafkaTopic(self, topic: str):
        broker = f"{self.kIp}:{self.kPort}"
        client = AdminClient({'bootstrap.servers': broker})
        fs = client.delete_topics([topic])

        for t, f in fs.items():
            try:
                f.result()
                log.debug(f"External Connector: Topic {t} deleted")
                return t
            except Exception as e:
                log.error(f"External Connector: Failed to delete topic {t}: {e}")
                return 0

    def createKafkaConsumer(self, group_id: str, topic: str) -> Consumer:
        conf = {
            'bootstrap.servers': f"{self.kIp}:{self.kPort}",
            'group.id': group_id,
            'auto.offset.reset': 'latest'
        }
        consumer = Consumer(conf)
        consumer.subscribe([topic])
        log.debug(f"External Connector: Kafka consumer enabled for topic {topic}")
        return consumer

    def createKafkaProducer(self) -> Producer:
        conf = {
            'bootstrap.servers': f"{self.kIp}:{self.kPort}"
        }
        return Producer(conf)


    ### Prometheus methods ###

    def startPrometheusJob(self, vnfdId: str, nsId: str, period: int, job_id: str):
        header = {'Accept': 'application/json', 'Content-Type': 'application/json'}
        uri = f"http://{self.monIp}:{self.monPort}/prom-manager/exporter"
        name = f"forecasting-{nsId}-{vnfdId}"
        metric = f"/metrics/{nsId}/{vnfdId}"
        body = {
            "name": name,
            "endpoint": [{"address": self.localIp, "port": self.localPort}],
            "vnfdId": vnfdId,
            "nsId": nsId,
            "collectionPeriod": period,
            "metrics_path": metric,
            "forecasted": "yes",
            "honor_labels": "true",
            "exporter": "forecasting_exporter"
        }
        log.debug(f"External Connector: Prometheus job request \n{body}")
        try:
            conn = HTTPConnection(self.monIp, int(self.monPort))
            conn.request("POST", uri, body=json.dumps(body), headers=header)
            resp = conn.getresponse()
            data = resp.read().decode("utf-8")
            reply = json.loads(data)
            conn.close()
            log.debug(f"EC: Prometheus job reply: {reply}")
            return reply.get('exporterId')
        except ConnectionRefusedError:
            log.error("EC: Error, connection refused")
            return None

    def stopPrometheusJob(self, jobId: str):
        header = {'Accept': 'application/json', 'Content-Type': 'application/json'}
        path = f"http://{self.monIp}:{self.monPort}/prom-manager/exporter/{jobId}"
        log.debug(f"External Connector: Deleting prometheus job {jobId}")
        try:
            conn = HTTPConnection(self.monIp, int(self.monPort))
            conn.request("DELETE", path, headers=header)
            rsp = conn.getresponse()
            log.debug(f"External Connector: Deleted prometheus job reply: {rsp.status}")
        except ConnectionRefusedError:
            log.error("External Connector: Error, connection refused")


    ### SCRAPER methods ###

    def startScraperJob(self, nsid: str, topic: str, vnfdid: str, metric: str,
                        expression: str, period: int):
        header = {'Accept': 'application/json', 'Content-Type': 'application/json'}
        body = {
            "nsid": nsid,
            "vnfid": vnfdid,
            "interval": period,
            "performanceMetric": metric,
            "kafkaTopic": topic,
            "expression": expression
        }
        log.debug(f"External Connector: Scraper job request \n{body}")
        path = f"http://{self.monIp}:{self.monPort}/prom-manager/prometheus_scraper"
        try:
            conn = HTTPConnection(self.monIp, int(self.monPort))
            conn.request("POST", path, body=json.dumps(body), headers=header)
            re = conn.getresponse()
            reply = json.loads(re.read().decode("utf-8"))
            conn.close()
            log.debug(f"External Connector: Scraper job reply \n{reply}")
            return reply.get('scraperId')
        except ConnectionRefusedError:
            log.error("External Connector: Error, connection refused")
            return None

    def stopScraperJob(self, job_id: str):
        header = {'Accept': 'application/json'}
        path = f"http://{self.monIp}:{self.monPort}/prom-manager/prometheus_scraper/{job_id}"
        log.debug(f"External Connector: Deleting scraper job {job_id}")
        try:
            conn = HTTPConnection(self.monIp, int(self.monPort))
            conn.request("DELETE", path, headers=header)
            rsp = conn.getresponse()
            log.debug(f"External Connector: Deleted scraper job reply: {rsp.status}")
            conn.close()
            return True
        except ConnectionRefusedError:
            log.error("External Connector: Error, connection refused")
            return False
