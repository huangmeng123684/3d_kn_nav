import asyncio
import time

async def fetch_data(name: str, delay: int) -> str:
    """模拟异步IO操作：等待指定的秒数后返回结果"""
    print(f"[{time.strftime('%X')}] 任务 {name}：开始请求")
    await asyncio.sleep(delay)  # 模拟网络IO，协程在此挂起
    print(f"[{time.strftime('%X')}] 任务 {name}：请求完成")
    return f"任务{name}的结果"

async def main() -> None:
    # 1. 串行执行（直接 await 协程）
    print(f"[{time.strftime('%X')}] === 串行执行 ===")
    result1 = await fetch_data("A", 1)  # 先执行A，挂起main直至A完成
    result2 = await fetch_data("B", 1)  # A完成后，再执行B
    print(f"串行结果：{result1}，{result2}")

    # 2. 并发执行（使用 Task）
    print(f"\n[{time.strftime('%X')}] === 并发执行 ===")
    # 创建两个Task，协程被提交到事件循环，立即进入就绪队列
    task1 = asyncio.create_task(fetch_data("C", 2))
    task2 = asyncio.create_task(fetch_data("D", 1))
    
    # 同时等待两个Task完成（并发执行，总耗时约2秒而非3秒）
    results = await asyncio.gather(task1, task2)
    print(f"并发结果：{results[0]}，{results[1]}")

    # 3. 创建Task但不立即等待（后台运行）
    print(f"\n[{time.strftime('%X')}] === 后台任务 ===")
    task3 = asyncio.create_task(fetch_data("E", 3))
    # 先执行其他操作
    await asyncio.sleep(0.5)
    print(f"[{time.strftime('%X')}] main：完成其他操作")
    # 最后再等待后台任务完成
    result3 = await task3
    print(f"后台任务结果：{result3}")

# 程序入口：run 创建并驱动事件循环
if __name__ == "__main__":
    asyncio.run(main())