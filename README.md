# turtlesim_autodraw


## `roscpp` port

 This is the package adapted from [turtlesim_cleaner](https://github.com/cbsudux/turtlesim_cleaner) ROS tutorial project written with `rospy`. I ported this into `roscpp`, and expanded some functionalities.  
 `rospy`를 사용한 [turtlesim_cleaner](https://github.com/cbsudux/turtlesim_cleaner) 패키지를 C++(`roscpp`)로 포팅 후 추가 기능을 더한 ROS 튜토리얼 패키지입니다.

## Changes from original project(`turtlesim_cleaner`)

* `rospy` codes that is not identically available on `roscpp` has been redone in my descretion.  
`rospy`와 `roscpp`의 차이로 인해 완벽하게 동일하게 포팅될 수 없는 부분을 임의로 보완하였습니다.

* Names of files, variables and functions has been altered for easier understanding of each file's behavior.  
파일, 변수, 함수 등의 이름을 한 번에 그 용도를 알기 쉽도록 변경했습니다.

* Description of code has been added in codes as comments(in Korean, for now).  
각 부분별로 한국어 주석 설명을 추가하였습니다.

* `*.launch` file now includes arguments.  
`*.launch` 파일에서 argument 설정이 추가되었습니다.

* Client nodes called by `*.launch` files will now be automatically respawned right after client node finishes.  
`*.launch` 파일로 실행된 클라이언트 노드가 실행을 모두 마친 후 자동으로 재시작(respawn)됩니다.

## TODO

* DrawCircle client & server - requires porting from `rospy` to `roscpp`.
* DrawDiagonal - new node that draws not only square but also N-th diagonals with N given.