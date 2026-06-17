from __future__ import annotations
import typing
import xCoreSDK_python
__all__: list[str] = ['BaseEthercat', 'SDOData', 'SlaveInfo']
class BaseEthercat:
    def GetSlaveCount(self, ec: dict) -> int:
        """
        获取从站数量
        
        Args:
            ec (dict): 错误码输出
        
        Returns:
            int: 从站数量
        """
    def GetSlaveInfo(self, slave_addr: typing.SupportsInt, ec: dict) -> ...:
        """
        获取从站信息
        
        Args:
            slave_addr: 从站地址
            ec (dict): 错误码输出
        
        Returns:
            从站信息
        """
    def GetSlaveState(self, slave_addr: typing.SupportsInt, ec: dict) -> int:
        """
        获取某个从站状态
        
        Args:
            slave_addr: 从站地址
            ec (dict): 错误码输出
        
        Returns:
            从站状态
        """
    def GetSlavesInfo(self, ec: dict) -> list[...]:
        """
        获取所有从站信息
        
        Args:
            ec (dict): 错误码输出
        
        Returns:
            所有从站信息
        """
    def ReadPDO(self, slave_addr: typing.SupportsInt, offset: typing.SupportsInt, size: typing.SupportsInt, data: list, ec: dict) -> bool:
        """
        读PDO
        
        Args:
            slave_addr: 从站地址
            offset: pdo偏移
            size: pdo长度
            data: 数据
            ec (dict): 错误码输出
        
        Returns:
            bool: 是否成功
        """
    def ReadSDO(self, slave_addr: typing.SupportsInt, index: typing.SupportsInt, sub_index: typing.SupportsInt, length: typing.SupportsInt, data: list, over_time: typing.SupportsInt, ec: dict) -> bool:
        """
        读SDO
                            
        Args:
            slave_addr: 从站地址
            index: 索引
            sub_index: 子索引
            length: 长度
            data: 数据
            over_time: 超时时间
            ec (dict): 错误码输出
        
        Returns:
            bool: 是否成功
        """
    def SetSlavesState(self, state: typing.SupportsInt, ec: dict) -> bool:
        """
        从站全部切状态
        
        Args:
            state: 从站状态
            ec (dict): 错误码输出
        
        Returns:
            bool: 是否成功
        """
    def WriteMultiSDO(self, slave_addr: typing.SupportsInt, SDO_data: list, ec: dict) -> bool:
        """
        写多个SDO
        
        Args:
            slave_addr: 从站地址
            SDO_data: 数据
            ec (dict): 错误码输出
        
        Returns:
            bool: 是否成功
        """
    def WritePDO(self, slave_addr: typing.SupportsInt, offset: typing.SupportsInt, size: typing.SupportsInt, data: list, ec: dict) -> bool:
        """
        写PDO
        
        Args:
            slave_addr: 从站地址
            offset: pdo偏移
            size: pdo长度
            data: 数据
            ec (dict): 错误码输出
        
        Returns:
            bool: 是否成功
        """
    def WriteSDO(self, slave_addr: typing.SupportsInt, index: typing.SupportsInt, sub_index: typing.SupportsInt, length: typing.SupportsInt, data: list, over_time: typing.SupportsInt, ec: dict) -> bool:
        """
        写SDO
        
        Args:
            slave_addr: 从站地址
            index: 索引
            sub_index: 子索引
            length: 长度
            data: 数据
            over_time: 超时时间
            ec (dict): 错误码输出
        
        Returns:
            bool: 是否成功
        """
    def __init__(self, arg0: xCoreSDK_python.BaseRobot) -> None:
        ...
class SDOData:
    """
    SDO数据信息
    """
    data: list
    def __init__(self) -> None:
        ...
    @property
    def index(self) -> int:
        ...
    @index.setter
    def index(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def length(self) -> int:
        ...
    @length.setter
    def length(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def over_time(self) -> int:
        ...
    @over_time.setter
    def over_time(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def print_data(self) -> int:
        ...
    @print_data.setter
    def print_data(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def sub_index(self) -> int:
        ...
    @sub_index.setter
    def sub_index(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def wait_time(self) -> int:
        ...
    @wait_time.setter
    def wait_time(self, arg0: typing.SupportsInt) -> None:
        ...
class SlaveInfo:
    """
    机器人从站信息
    """
    slaveName: str
    def __init__(self) -> None:
        ...
    @property
    def alStatus(self) -> int:
        ...
    @alStatus.setter
    def alStatus(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def productCode(self) -> int:
        ...
    @productCode.setter
    def productCode(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def reversionNumber(self) -> int:
        ...
    @reversionNumber.setter
    def reversionNumber(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def slaveAddr(self) -> int:
        ...
    @slaveAddr.setter
    def slaveAddr(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def slaveId(self) -> int:
        ...
    @slaveId.setter
    def slaveId(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def vendorId(self) -> int:
        ...
    @vendorId.setter
    def vendorId(self, arg0: typing.SupportsInt) -> None:
        ...
