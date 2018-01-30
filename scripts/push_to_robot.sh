
ROOT_PATH=`dirname $0`/..
MAGIC2="$ROOT_PATH/magic2"

if [ "$#" -ne "2" ]; then
    echo "Usage: $0 <robot_id> <groundstation_ip>"
    exit 1
fi

ROBOTID=$1
GROUNDSTATION_IP=$2
IP="10.0.$ROBOTID.1"

echo "Pushing to robot $ROBOTID at $IP from $GROUNDSTATION_IP"

# Create files for current commit hash and local diff from HEAD (tracked files only for now)
version_file=$(readlink -f $ROOT_PATH/config/git-version)
diff_file=$(readlink -f $ROOT_PATH/config/git-diff)
git log --oneline --no-walk HEAD > $version_file
git --no-pager diff `git symbolic-ref --short HEAD` -- > $diff_file

STAGE=/tmp/magic_stage
rm -rf $STAGE
mkdir -p $STAGE/magic2/bin $STAGE/magic2/config $STAGE/magic2/scripts \
            $STAGE/magic2/lcmtypes $STAGE/magic2/setup $STAGE/magic2/resc \
            $STAGE/magic2/april2/web

cp -R $ROOT_PATH/bin $ROOT_PATH/config $MAGIC2/scripts $MAGIC2/setup $MAGIC2/resc $STAGE/magic2
cp -R $MAGIC2/april2/web $STAGE/magic2/april2/
cp -R $ROOT_PATH/lcmtypes/* $MAGIC2/lcmtypes/* $MAGIC2/april2/lcmtypes/* $STAGE/magic2/lcmtypes

# selecting appropriate config files
if [ -f $ROOT_PATH/config/robot-$ROBOTID.config ]
then
    cp $ROOT_PATH/config/robot-$ROBOTID.config $STAGE/magic2/config/robot.config
else
    echo "WARNING, NO PER-ROBOT CONFIG FILE FOUND $ROOT_PATH/config/robot-$ROBOTID.config"
    cp $ROOT_PATH/config/robot-gen.config $STAGE/magic2/config/robot.config
fi

if [ -f $ROOT_PATH/config/proc-robot-$ROBOTID.config ]
then
    cp $ROOT_PATH/config/proc-robot-$ROBOTID.config $STAGE/magic2/config/proc-robot.config
else
    echo "WARNING, NO PER-ROBOT PROCMAN FILE FOUND $ROOT_PATH/config/proc-robot-$ROBOTID.config"
    cp $ROOT_PATH/config/proc-robot-gen.config $STAGE/magic2/config/proc-robot.config
fi

# Create the groundstation IP configuration file
groundstation_file=$(readlink -f $STAGE/magic2/config/groundstation)
echo "Ground Station File: $groundstation_file"
echo "Ground Station IP: $GROUNDSTATION_IP"
echo $GROUNDSTATION_IP > $groundstation_file

ssh april@$IP '/home/april/magic2/scripts/procman_stop.sh'
ssh april@$IP sudo date `date +%m%d%H%M%Y.%S`
rsync -rzP --delete $STAGE/magic2 april@$IP:/home/april/
ssh april@$IP '/home/april/magic2/scripts/procman_start.sh'
