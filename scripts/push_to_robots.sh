
MAGIC2=`dirname $0`/..
groundstation_ip=$(ifconfig -a | grep -F "10.0." | head -n 1 | perl -l -ne '/[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/ && print $&')
if [ -z "$groundstation_ip" ]; then
    echo "\nYou don't have a 10.0.X.Y IP address.\nEnsure you are connected to a MAGIC network.\nAborting..."
    exit
fi

echo "USING MAGIC2=$MAGIC2"

for robot_id in "$@"
do
    $MAGIC2/scripts/push_to_robot.sh $robot_id $groundstation_ip
done
