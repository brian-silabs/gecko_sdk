# automatically generated by the FlatBuffers compiler, do not modify

# namespace: 

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class Entry(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAs(cls, buf, offset=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = Entry()
        x.Init(buf, n + offset)
        return x

    @classmethod
    def GetRootAsEntry(cls, buf, offset=0):
        """This method is deprecated. Please switch to GetRootAs."""
        return cls.GetRootAs(buf, offset)
    # Entry
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Entry
    def Key(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # Entry
    def ValueType(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint8Flags, o + self._tab.Pos)
        return 0

    # Entry
    def Value(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            from flatbuffers.table import Table
            obj = Table(bytearray(), 0)
            self._tab.Union(obj, o)
            return obj
        return None

def Start(builder): builder.StartObject(3)
def EntryStart(builder):
    """This method is deprecated. Please switch to Start."""
    return Start(builder)
def AddKey(builder, key): builder.PrependUOffsetTRelativeSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(key), 0)
def EntryAddKey(builder, key):
    """This method is deprecated. Please switch to AddKey."""
    return AddKey(builder, key)
def AddValueType(builder, valueType): builder.PrependUint8Slot(1, valueType, 0)
def EntryAddValueType(builder, valueType):
    """This method is deprecated. Please switch to AddValueType."""
    return AddValueType(builder, valueType)
def AddValue(builder, value): builder.PrependUOffsetTRelativeSlot(2, flatbuffers.number_types.UOffsetTFlags.py_type(value), 0)
def EntryAddValue(builder, value):
    """This method is deprecated. Please switch to AddValue."""
    return AddValue(builder, value)
def End(builder): return builder.EndObject()
def EntryEnd(builder):
    """This method is deprecated. Please switch to End."""
    return End(builder)