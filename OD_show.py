import canopen

network = canopen.Network()
# network.connect(bustype='socketcan', channel='can0')
network.connect(bustype='ixxat', channel=0, bitrate=1000000)
# This will attempt to read an SDO from nodes 1 - 127
node = network.add_node(1, 'od.eds')
for obj in node.object_dictionary.values():
    print('0x%X: %s' % (obj.index, obj.name))
    if isinstance(obj, canopen.objectdictionary.Record):
        for subobj in obj.values():
            print('  %d: %s' % (subobj.subindex, subobj.name))
network.disconnect()
