# C++11 语法新特性



## C++ STL prev()函数

1.直观理解

```C++
new_iterator = prev(iterator，n)
```

当“n“为正数时，返回传入迭代器“iterator”左边，距离”iterator“ n个单位的迭代器”new_iterator“。

当“n“为负数时，返回传入迭代器“iterator”右边，距离”iterator“ n个单位的迭代器"new_iterator"。

```C++
new_iterator = prev(iterator)
```

不写n的话，默认向“iterator”左边移动1个单位。

如果是随机访问迭代器，就只执行一次运算符操作 +=n( -=n )，否则，执行n次持续的递减或递增操作 ++(--)。

2.prev()的函数原型

```C++
template< class BidirIt >BidirIt prev(
  BidirIt it, 
  typename std::iterator_traits<BidirIt>::difference_type n = 1 );
 
template< class BidirIt >constexpr BidirIt prev(
  BidirIt it, 
  typename std::iterator_traits<BidirIt>::difference_type n = 1 );
```

**prev()的含义：** 表示迭代器左移n个单位，即迭代器-n。 若移动n个单位的步长后，超出迭代器范围`[begin,end)`，则此行为未定义。

**prev()函数返回值：** 函数的返回值为一个迭代器，也就是传入迭代器左移n个单位后，返回这个移动后的新迭代器。

**3.测试代码：**

```C++
 vector<int> vec{ 1,2,3,4,5,6,7 };
 vector<int>::iterator end = vec.end();
 for (int i = 1; i <= vec.size(); ++i)
 {
  auto it = prev(end, i);
  cout << "end左移" << i << "个单位后的元素值为：" << *it << endl;
 } 
```

**测试结果：**

```C++
end左移1个单位后的元素值为：7
end左移2个单位后的元素值为：6
end左移3个单位后的元素值为：5
end左移4个单位后的元素值为：4
end左移5个单位后的元素值为：3
end左移6个单位后的元素值为：2
end左移7个单位后的元素值为：1
```