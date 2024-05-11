#!/bin/bash

# Read the WEB_SERVER environment variable
web_server=${WEB_SERVER:-"no"}

if [ "$web_server" == "yes" ]; then
    # If WEB_SERVER is "yes", run the Flask server
    export FLASK_APP=server.py
    export FLASK_ENV=development
    flask run --host=0.0.0.0 --port=5000

else
    # If WEB_SERVER is "no", run the client code
    server_url=${SERVER_URL:-"http://127.0.0.1:5000"}
    window_size=${WINDOW_SIZE:-"50"}
    topics=${TOPICS:-"/scan /joint_states /go1_controller/odom"}

    echo "server_url: $server_url"
    echo "window_size: $window_size"
    echo "topics: $topics"

    # Convert the topics string into an array
    IFS=' ' read -r -a topics_array <<< "$topics"

    # Use "${topics_array[@]}" to expand the topics into separate arguments
    python3 client.py "${topics_array[@]}" --server_url $server_url --window_size $window_size
fi
