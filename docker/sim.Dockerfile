# Minimal container for sim/fake_fc.py - just Python + pymavlink, no ROS2.
FROM python:3.11-slim

RUN pip install --no-cache-dir pymavlink==2.4.43 pyserial

WORKDIR /sim
COPY sim/fake_fc.py .

# -u: unbuffered stdout so `docker compose logs` is live.
CMD ["python3", "-u", "fake_fc.py"]
