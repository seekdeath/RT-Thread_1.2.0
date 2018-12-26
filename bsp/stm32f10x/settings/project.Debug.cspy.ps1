param([String]$debugfile = "");

# This powershell file has been generated by the IAR Embedded Workbench
# C - SPY Debugger, as an aid to preparing a command line for running
# the cspybat command line utility using the appropriate settings.
#
# Note that this file is generated every time a new debug session
# is initialized, so you may want to move or rename the file before
# making changes.
#
# You can launch cspybat by typing Powershell.exe -File followed by the name of this batch file, followed
# by the name of the debug file (usually an ELF / DWARF or UBROF file).
#
# Read about available command line parameters in the C - SPY Debugging
# Guide. Hints about additional command line parameters that may be
# useful in specific cases :
#   --download_only   Downloads a code image without starting a debug
#                     session afterwards.
#   --silent          Omits the sign - on message.
#   --timeout         Limits the maximum allowed execution time.
#


if ($debugfile -eq "")
{
& "D:\IAR Systems\Embedded Workbench 8.0\common\bin\cspybat" -f "C:\Users\oboth.oboth-PC\Desktop\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]源码\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\stepmotor-both201401129\RT-Thread_1.2.0\bsp\stm32f10x\settings\project.Debug.general.xcl" --backend -f "C:\Users\oboth.oboth-PC\Desktop\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]源码\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\stepmotor-both201401129\RT-Thread_1.2.0\bsp\stm32f10x\settings\project.Debug.driver.xcl" 
}
else
{
& "D:\IAR Systems\Embedded Workbench 8.0\common\bin\cspybat" -f "C:\Users\oboth.oboth-PC\Desktop\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]源码\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\stepmotor-both201401129\RT-Thread_1.2.0\bsp\stm32f10x\settings\project.Debug.general.xcl" --debug_file=$debugfile --backend -f "C:\Users\oboth.oboth-PC\Desktop\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]源码\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\[j002]STM32步进电机高效S型T梯形曲线SpTA加减速控制算法\stepmotor-both201401129\RT-Thread_1.2.0\bsp\stm32f10x\settings\project.Debug.driver.xcl" 
}
