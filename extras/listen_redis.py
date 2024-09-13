import argparse
import redis

REDIS_HOST = "localhost"
REDIS_PORT = "6379"

def main(redis_key):
    redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
    while True:
        print(redis_client.get(redis_key))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("redis_key", type=str)
    args = parser.parse_args()
    main(args.redis_key)