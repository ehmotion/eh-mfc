#BeagleBone Black
#sudo ./usb-proxy --device=musb-hdrc.0 --driver=musb-hdrc --vendor_id=0eb7 --product_id=0e07 --mfcxtractor_ip=192.168.0.43

#Rpi4
#sudo ./usb-proxy --device=fe980000.usb --driver=fe980000.usb --vendor_id=0eb7 --product_id=0e07 --mfcxtractor_ip=192.168.0.43

#while true; do sudo ./usb-proxy --device=musb-hdrc.0 --driver=musb-hdrc --vendor_id=0eb7 --product_id=0e07 --mfcxtractor_ip=192.168.0.43; echo "waiting for wheel.."; sleep 5; done

cp -f opipc/raw_gadget.ko /opt/mfc/usbproxy/opipc/

cd /opt/mfc/usbproxy

if [ $(id -u) -ne 0 ]                                                                                                                                                                                                                                                          
  then echo "Please run as root"                                                                                                                                                                                                                                               
  exit                                                                                                                                                                                                                                                                         
fi                                                                                                                                                                                                                                                                             
                                                                                                                                                                                                                                                                               
if [ $(lsmod | grep raw_gadget | wc -l) -eq 0 ]
then
  cd opipc
  sudo ./insmod.sh
  cd ..
fi

while true
do
  if [ $(lsusb | grep 0eb7:0e07 | wc -l) -eq 0 ]
  then
    echo "#$(date):waiting for wheel.."
    sleep 5
  else
    #! sudo modprobe usbmon 2> /dev/null && die "Failed to load usbmon."
    #sudo tcpdump -i usbmon0 -w ./usb-proxy.pcap 2> ./tcpdump.log &
    #PID=$!

    #./usb-proxy --device=musb-hdrc.4.auto --driver=musb-hdrc --vendor_id=0eb7 --product_id=0e07 --mfcxtractor_ip=192.168.0.43
    ./usb-proxy --device=musb-hdrc.4.auto --driver=musb-hdrc --vendor_id=0eb7 --product_id=0e07 --mfcxtractor_ip=192.168.68.109

    #sudo kill -SIGINT $PID 2> /dev/null
  fi
done
