# Use a Python runtime matching the venv
FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1
# Set the working directory in the container
WORKDIR /app

# Install system dependencies for Pygame and Zenoh
RUN apt-get update && apt-get install -y \
    libsdl2-2.0-0 \
    libsdl2-image-2.0-0 \
    libsdl2-mixer-2.0-0 \
    libsdl2-ttf-2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (versions match .f1tenthvenv)
COPY requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt

# Copy the current directory contents into the container at /app
COPY . .

# Set environment variable to allow pygame to run without a monitor/video device
ENV SDL_VIDEODRIVER=dummy
ENV PYTHONUNBUFFERED=1

# Run the script
CMD ["python", "transmit_driving_inputs.py"]
