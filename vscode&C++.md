# VSCode使用笔记

安装插件CMake和CMake Tools

Ctrl+Shift+p

打开VSCode的指令面板，

然后输入cmake:q，VSCode会根据输入自动提示，然后选择CMake: Quick Start





`template<typename T>` 

C++中的模板声明语法。它表示在函数或类的定义中，使用了一个模板参数 `T`。

模板是一种通用的编程工具，允许我们编写可以适用于多种类型的代码。

通过使用模板，我们可以在不必为每种类型都编写不同的代码的情况下，实现代码的重用和泛化

模板函数的实现不能放在源文件中，需要直接放在头文件中。



`pcl::PointCloud<pcl::PointXYZI>& _cloud_in;` 和 `pcl::PointCloud<pcl::PointXYZI> _cloud_in;` 是两种不同的声明方式，分别表示引用和对象。

1. `pcl::PointCloud<pcl::PointXYZI>& _cloud_in;` 声明了一个名为 `_cloud_in` 的引用变量，它引用了 `pcl::PointCloud<pcl::PointXYZI>` 类型的对象。这意味着 `_cloud_in` 不是一个独立的对象，它只是一个别名，指向其他已经存在的 `pcl::PointCloud<pcl::PointXYZI>` 对象。因此，在声明时必须将 `_cloud_in` 初始化为引用某个有效的点云对象，否则会导致编译错误。
2. `pcl::PointCloud<pcl::PointXYZI> _cloud_in;` 声明了一个名为 `_cloud_in` 的对象，它是 `pcl::PointCloud<pcl::PointXYZI>` 类型的实际对象。这意味着 `_cloud_in` 是一个独立的点云对象，它拥有自己的存储空间和数据。在声明时，会自动调用 `pcl::PointCloud<pcl::PointXYZI>` 的构造函数来创建 `_cloud_in` 对象，并根据构造函数的参数进行初始化。



`pcl::PointCloud<pcl::PointXYZI>::Ptr` 是一个指向 `pcl::PointCloud<pcl::PointXYZI>` 对象的共享指针类型，它没有 `size()` 成员函数。要获取点云的大小，需要使用 `pcl::PointCloud<pcl::PointXYZI>::size()` 函数。

```c++
std::vector<double> depth(cloud_in->size());
```



