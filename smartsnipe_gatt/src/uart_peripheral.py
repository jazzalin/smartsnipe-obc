# Based on bluez example-advertisement and example-gatt-server
# Further information: https://scribles.net/creating-ble-gatt-server-uart-service-on-raspberry-pi/
# Adapted to work with ROS 
import sys
import rospy
import dbus, dbus.mainloop.glib
from gi.repository import GLib
from advertisement import Advertisement
from advertisement import register_ad_cb, register_ad_error_cb
from gatt_server import Service, Characteristic
from gatt_server import register_app_cb, register_app_error_cb
from smartsnipe_msgs import BoardState
 
BLUEZ_SERVICE_NAME =           'org.bluez'
DBUS_OM_IFACE =                'org.freedesktop.DBus.ObjectManager'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
GATT_MANAGER_IFACE =           'org.bluez.GattManager1'
GATT_CHRC_IFACE =              'org.bluez.GattCharacteristic1'
UART_SERVICE_UUID =            '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
UART_RX_CHARACTERISTIC_UUID =  '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
UART_TX_CHARACTERISTIC_UUID =  '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
LOCAL_NAME =                   'OBC-UART'
mainloop = None
 
class TxCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(self, bus, index, UART_TX_CHARACTERISTIC_UUID,
                                ['notify'], service)
        self.notifying = False
        GLib.io_add_watch(sys.stdin, GLib.IO_IN, self.on_console_input)
 
    def on_console_input(self, fd, condition):
        s = fd.readline()
        if s.isspace():
            pass
        else:
            self.send_tx(s)
        return True
 
    def send_tx(self, s):
        if not self.notifying:
            return
        value = []
        for c in s:
            value.append(dbus.Byte(c.encode()))
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': value}, [])
 
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
 
    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
 
class RxCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(self, bus, index, UART_RX_CHARACTERISTIC_UUID,
                                ['write'], service)
 
    def WriteValue(self, value, options):
        print('remote: {}'.format(bytearray(value).decode()))
 
class UartService(Service):
    def __init__(self, bus, index):
        Service.__init__(self, bus, index, UART_SERVICE_UUID, True)
        self.add_characteristic(TxCharacteristic(bus, 0, self))
        self.add_characteristic(RxCharacteristic(bus, 1, self))
 
class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
 
    def get_path(self):
        return dbus.ObjectPath(self.path)
 
    def add_service(self, service):
        self.services.append(service)
 
    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
        return response
 
class UartApplication(Application):
    def __init__(self, bus):
        Application.__init__(self, bus)
        self.add_service(UartService(bus, 0))
 
class UartAdvertisement(Advertisement):
    def __init__(self, bus, index):
        Advertisement.__init__(self, bus, index, 'peripheral')
        self.add_service_uuid(UART_SERVICE_UUID)
        self.add_local_name(LOCAL_NAME)
        self.include_tx_power = True

class UartHandler:
    
    def __init__(self):
        global mainloop
        self.main_loop = mainloop
        # Initialize dbus bluetooth interface
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.adapter = self.find_adapter()
        if not self.adapter:
            rospy.loginfo("BLE adapter not found. Exit initialization...")
            return

        self.service_manager = dbus.Interface(
                                self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter),
                                GATT_MANAGER_IFACE)
        self.ad_manager = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter),
                                    LE_ADVERTISING_MANAGER_IFACE)
 
        self.app = UartApplication(self.bus)
        self.adv = UartAdvertisement(self.bus, 0)
    
        self.main_loop = GLib.MainLoop()

        # Register service and advertisement
        self.service_manager.RegisterApplication(self.app.get_path(), {},
                                                 reply_handler=register_app_cb,
                                                 error_handler=register_app_error_cb)
        self.ad_manager.RegisterAdvertisement(self.adv.get_path(), {},
                                              reply_handler=register_ad_cb,
                                              error_handler=register_ad_error_cb)
    
    def find_adapter(self):
        remote_om = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                                   DBUS_OM_IFACE)
        objects = remote_om.GetManagedObjects()
        for o, props in objects.items():
            if LE_ADVERTISING_MANAGER_IFACE in props and GATT_MANAGER_IFACE in props:
                return o
            rospy.loginfo(('Skip adapter:', o))
        return None


    def run(self):
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            self.adv.Release()
        

# def find_adapter(bus):
#     remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
#                                DBUS_OM_IFACE)
#     objects = remote_om.GetManagedObjects()
#     for o, props in objects.items():
#         if LE_ADVERTISING_MANAGER_IFACE in props and GATT_MANAGER_IFACE in props:
#             return o
#         print('Skip adapter:', o)
#     return None
 
if __name__ == '__main__':
    uart = UartHandler()
    uart.run()