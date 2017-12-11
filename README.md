# polly_speech

## Overview

polly_speech package is text-to-speech engine using [Amazon Polly]. It support publish visemes that can using lip-sync for robots.

**Author(s): Byeong-Kyu Ahn <br/>
Maintainer: Byeong-Kyu Ahn, byeongkyu@gmail.com  <br/>
Affiliation: Robotics Research Group, The Unversity of Auckland**


## Installation

### Building from Source

#### Dependencies

- boto3: AWS SDK for Python http://aws.amazon.com/sdk-for-python/

        $ sudo pip install -U boto3

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        $ cd catkin_ws/src
        $ git clone https://github.com/byeongkyu/polly_speech.git
        $ catkin build

## Setup on AWS

You should join to AWS for using Amazon Polly. Visit https://us-east-2.console.aws.amazon.com/console and make account.
Next, go to [Identity and Access Management] and make users, get aws_access_key_id, aws_secret_access_key.

## Config File

Just copy config.yaml.template to config.yaml and fill the access information that you can get above


## Usage

        rosrun polly_speech polly_speech_node.py \_config_file:=path_to/config.yaml


## Nodes

### polly_speech_node

#### Subscribed Topics

* **`/speech_action`** ([polly_speech/SpeechAction])

        Text to speech.

#### Published Topics

* **`/lipsync_vowel`** ([std_msgs/String])

        String for lipsync

## Bugs & Feature Requests

Please report bugs and request features using the JIRA.


[Identity and Access Management]: https://console.aws.amazon.com/iam
[Amazon Polly]: https://us-east-2.console.aws.amazon.com/polly
