#!/bin/sh
# Author : Laurent Alloza

# Install Netbeans Project Files
echo "Copying project files."
mkdir -vp $HOME/NetBeansProjects
SRC=./ReactionWheel
DST=$HOME/NetBeansProjects
cp -R $SRC $DST
echo "Done."

# Documentation and Desktop Files
echo "Copying courseware files."
SRC=./Courseware
DST=$HOME/NetBeansProjects
cp -R $SRC $DST

chmod a+x $DST/Courseware/*.desktop
chmod a+x $DST/Courseware/interface/*.AppImage
gio set $DST/Courseware/netbeans.desktop "metadata::trusted" yes
gio set $DST/Courseware/interface.desktop "metadata::trusted" yes
gio set $DST/Courseware/xeno3alchemy.desktop "metadata::trusted" yes

echo "Icon="$DST"/Courseware/interface/Interface.png" >> $DST/Courseware/interface.desktop
echo "Exec=env http_proxy= "$DST"/Courseware/interface/Interface.AppImage" >> $DST/Courseware/interface.desktop

echo "Done."

