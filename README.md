# Omni-drive Path Tracking Controller

## 基本概念
### 主要運作流程
* 獲取目標點座標、當前機器人座標，傳入 A* service，獲取全局路徑（Global Path）
  
* 以 Rolling window method 求得 local goal，作為當前追蹤目標
  
* 計算速度向量長度
  
* 計算速度向量方向

* 機器人到達目標點座標後，追蹤角度

* 完成追蹤此路徑
  
<!-- ![picture 2](images/923cd467b62eba81b867434cb3f9ba48253c4265355caa06e7fd2e2f70eac9c3.png) -->

<img src="images/923cd467b62eba81b867434cb3f9ba48253c4265355caa06e7fd2e2f70eac9c3.png" alt="drawing" style="width:600px;"/>
