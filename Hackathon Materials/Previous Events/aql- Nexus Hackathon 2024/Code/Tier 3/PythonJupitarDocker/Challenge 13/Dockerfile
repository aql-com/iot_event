FROM jupyter/base-notebook

# Mount the requirements file into the container
ADD ./requirements.txt ./requirements.txt

# Install curl
# RUN apt-get -y update; apt-get -y install curl

USER root

# Install curl
RUN apt-get -y update && \
    apt-get -y install curl

USER $NB_UID

# Install any needed dependencies specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt
