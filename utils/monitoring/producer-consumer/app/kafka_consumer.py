import argparse
import json
import logging
from confluent_kafka import KafkaError, KafkaException
from confluent_kafka.error import ConsumeError
from kafkaConnections import kafkaConnections

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
log = logging.getLogger(__name__)

# Argument parser
parser = argparse.ArgumentParser(description="Kafka Consumer Script")
parser.add_argument('--file', dest='file', default="configC1.conf", help="Configuration file")
args = parser.parse_args()

# Initialize Kafka connection
try:
    ec = kafkaConnections(args.file)
    topic = ec.ktopic
    idx = ec.kID
    consumer = ec.createKafkaConsumer(idx, topic)
except Exception as e:
    log.error(f"Error initializing Kafka consumer: {e}")
    exit(1)

def consume_messages():
    """ Continuously polls and processes Kafka messages """
    try:
        while True:
            msg = consumer.poll(1.0)
            if msg is None:
                continue
            if msg.error():
                if msg.error().code() == KafkaError._PARTITION_EOF:
                    log.warning(f"Partition EOF reached: {msg.topic()} [{msg.partition()}] at offset {msg.offset()}")
                else:
                    raise KafkaException(msg.error())
            else:
                try:
                    loaded_json = json.loads(msg.value().decode("utf-8"))
                    log.info("[METRICS RECEIVED]\n" + json.dumps(loaded_json, indent=4, sort_keys=True))
                except json.JSONDecodeError as e:
                    log.error(f"JSON decoding error: {e} - Message: {msg.value()}")
    except ConsumeError as e:
        log.error(f"Consumer error: {e}")
    except KeyboardInterrupt:
        log.info("Consumer interrupted by user.")
    finally:
        log.info("Closing consumer.")
        consumer.close()

if __name__ == "__main__":
    consume_messages()
