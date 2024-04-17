#!/usr/bin/env python
from oasis.agent.oasis_agent import OasisAgent
from adapter import Adapter 

import json
import redis
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_DB = 1
REDIS_PASSWORD = "123456"


def get_task_info(task_id):
    pool = redis.ConnectionPool(host=REDIS_HOST, port=REDIS_PORT, db=REDIS_DB, password=REDIS_PASSWORD, decode_responses=True)
    r = redis.Redis(connection_pool=pool)
    task_info = r.hget("task_infos", task_id)
    task_info = json.loads(task_info)
    return task_info['task']

class DemoAgent(OasisAgent):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def start_process(self):
        # run adapter code
        # self.task_id
        task_info =get_task_info(self.task_id)
        Adapter(host=self.oasis_config.get('node'), port=2000, task_info=task_info).run()
        print("just adapter .. ")
        # self.ad_agent_server.launch()
        # self.ad_agent_server.shutdown()
        # self.ad_agent_server.launch_pure_pursuit() # 单独启动 pure_pursuit 模块, 名字可以自定义, 和 ads_agent 对应即可
        # self.ad_agent_server.shutdown_pure_pursuit()

    def stop_process(self, task_id):
        # stop adapter code
        super().stop_process(task_id)

if __name__ == "__main__":
    # 配置参数
    oasis_config = {
        "node": "127.0.0.1",       # 部署 OASIS 的机器 IP, 需要更改
        "node_name": "cluster_machine_1",
        # "name": "Oasis Driver_规控版",        # 自驾系统名字, 在前端配置后在此处填写, 需要更改
        "name": "UdiPnc",        # 自驾系统名字, 在前端配置后在此处填写, 需要更改
        "version": "1.0"           # 自驾系统版本, 在前端配置后在此处填写, 需要更改
    }
    ads_config = {
        "host": "localhost",       # ads_agent 所在的机器 IP, 需要更改, 可以填 None
        "port": 23456,             # ads_agent 开放的端口, 需要更改, 可以填 None
        "auto": False              # 是否需要 oasis_agent 自动调用 ads_agent, 需要更改
                                   # True 对应一套内置的调用顺序
                                   # 希望自主控制算法和 adapter 的启动顺序、并且在启动算法上有额外的需求的话, 请使用 False
                                   # 额外需求是指比如需要单独启动算法某个模块之类
    }

    DemoAgent(oasis_config=oasis_config, ads_agent_config=ads_config).run()