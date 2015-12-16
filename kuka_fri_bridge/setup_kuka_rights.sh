if [ -z "$1" ]
  then
    echo "No argument supplied, please provide desired username to run lwr_main without sudo"
    exit
fi

if [ "$(id -u)" != "0" ]; then
	echo "Sorry, you are not root."
	exit 1
fi

LIMITS=/etc/security/limits.conf
COMMON_SESSION=/etc/pam.d/common-session
echo "Looking into \"$LIMITS\": is the line \"$USER hard rtprio 95\"  present ?"

# Adding lines in /etc/security/limits.conf
DES_USERNAME=$1
if grep -Fxq "$DES_USERNAME hard rtprio 95" "$LIMITS"
	then
    echo "Settings already added in $LIMITS."
else
	echo "Settings not present. Adding the missing lines to $LIMITS."
	echo "# Settings to allow running lwr_main without sudo rights.
$DES_USERNAME hard rtprio 95
$DES_USERNAME soft rtprio 95
$DES_USERNAME hard memlock unlimited
$DES_USERNAME soft memlock unlimited" >> $LIMITS
fi


# Adding line in /etc/pam.d/common-session
if grep -Fxq "session required pam_limits.so" $COMMON_SESSION
	then
    echo "Settings already added in $COMMON_SESSION."
else
	echo "Settings not present. Adding the missing line to $COMMON_SESSION."
	echo "# To run lwr_main without sudo rights.
session required pam_limits.so" >> $COMMON_SESSION
fi

echo "Settings are now ok. Reboot to make it work".
echo "Test the success with command 'ulimit -r -l': settings should be '95' and 'unlimited'"
