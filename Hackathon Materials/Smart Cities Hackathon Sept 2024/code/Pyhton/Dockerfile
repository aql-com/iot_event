# Use an official Python runtime as a parent image
FROM python:3.10-slim

# Set the working directory in the container
WORKDIR /app

# Copy the requirements file into the container at /app
COPY requirements.txt /app/


# Update Paho MQTT library to the latest version
RUN pip install --no-cache-dir paho-mqtt --upgrade

# Install any needed dependencies specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Copy all Python files from the host's hackathon_javascript folder into the container's /app folder
COPY hackathon/*.py /app/

# Default command to run when the container starts
CMD ["python", "hello.py"]
