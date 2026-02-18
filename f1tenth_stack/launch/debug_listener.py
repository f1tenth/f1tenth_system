import zenoh
import time

# 1. Create Config
conf = zenoh.Config()

# 2. CRITICAL FIX: Tell this script exactly where the Laptop is.
# REPLACE '10.13.178.75' with your Laptop's current IP if it changed.
conf.insert_json5("connect/endpoints", '["tcp/10.13.178.75:7447"]')

# 3. Open Session
print(f"Connecting to Laptop...")
session = zenoh.open(conf)

# 4. Define Callback
def listener(sample):
    # Convert ZBytes to standard bytes
    data = sample.payload.to_bytes()
    
    # NOW you can slice it
    print(f"Received {len(data)} bytes | Data: {list(data[:10])}...")

# 5. Subscribe
print("Subscribed to 'rt/joy'. Waiting for data...")
sub = session.declare_subscriber("rt/joy", listener)

while True:
    time.sleep(1)