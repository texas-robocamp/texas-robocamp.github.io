---
title: "Arrays"
tags: [c++]
keywords: c++
last_updated: July 2, 2018
summary:
sidebar: tutorials
permalink: arrays.html
---

Sometimes we want to store lists of data together in one place. For example, it might be handy to have a list of all your grades from science class. You could go back later and average all the grades in the list together to find out your final grade in science. In C++, lists are called arrays, and they have some special properties.

You can think of an array like a motel. Let's begin by imagining a single-story motel complex. The rooms are laid out in a row right next to each other. If you stand in front of room 7, you know that room 8 will be right next door, and room 9 will be after room 8. Arrays work in the same way. We can put values inside each of the "rooms" in the array, and since they are numbered logically, we can quickly find any "room" we need.

When you build a motel, you have to decide ahead of time how many rooms you want there to be, and you can't change your mind after the motel is built (without paying a lot of construction costs). In a C++ array, at the beginning of the function you declare the type, name, and size of your array. Going back to the science grade example, we would declare a science grade array like this:

`float gradeArray[5];`

The brackets with the number mean that we are constructing an array that is five units in length named gradeArray. We declare it as a float because we plan on storing floating point numbers in each of the "rooms" in the array. We can substitute the number 5 with any length of array we want, but once we declare our array to be a certain size, we can't change it.

Now that we've declared an array, we need to put things inside it. Inside our main function, we'll enter five grades into the array. When you put a value into an array, you have to tell the computer where to put it, just like the front desk in a motel would assign you a room. Like rooms in a motel, locations, or elements, in an array are numbered—in an array, though, the numbers are from 0 to one less than the size of the array. In gradeArray, then, there are five elements, which are numbered 0, 1, 2, 3, and 4. We talk about a specific elements of an array by writing the name of the array and then putting the location number in brackets, like this: gradeArray[0] . Let's put some values into them in the following code:

```cpp
gradeArray[0] = 98.3;
gradeArray[1] = 91.1;
gradeArray[2] = 89.8;
gradeArray[3] = 90.5;
gradeArray[4] = 95.0;
```

Now that we have our grades in an array, we want to calculate our final grade by averaging them all together. To do so, we need to visit each element in the array (imagine going from door to door in a motel) and add together the values in each position. Finally, we need to divide them by the number of values in the array. To do this systematically, we can use a for loop. We will loop through each element in the array and add its value to a variable called gradeSum that we initialize ahead of time. After we finish the loop, we will divide gradeSum by 5 to get the average. Here's the code for this program:

```cpp
#include <iostream>
using namespace std;

int main(){
   //declare the array and the other variables
   float gradeArray[5];
   float gradeSum, gradeAverage;
   int i;

   //put some grades into the array
   gradeArray[0] = 98.3;
   gradeArray[1] = 91.1;
   gradeArray[2] = 89.8;
   gradeArray[3] = 90.5;
   gradeArray[4] = 95.0;

   //initialize the gradeSum variable to be 0
   gradeSum = 0;

   //loop through each location in gradeArray (there are 5)
   for(i=0; i<5; i++){
      //add the value of the thing stored at location
      //i in gradeArray to gradeSum
      gradeSum += gradeArray[i];
   }

   //to finish the average, divide gradeSum by 5
   gradeAverage = gradeSum/5;

   //print out the average
   cout << "Your science class average is " << gradeAverage << ".\n";
}
```

### Task 7.1:

Change the program above so that it stores and uses 7 grades instead of 5 to calculate the average.

Yay! Two more :) Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!

### Task 7.2:

Write a program that asks a user to enter 10 numbers, puts those numbers in an array, and then prints out the numbers in the array backwards? Hint: use a loop that counts backwards.

Almost there! Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!

One really cool feature of arrays is that you can have multidimensional arrays. Let's return to the motel example from earlier, where we had a one-story motel. Imagine that we add a second floor to the motel of exactly the same length as the ground floor. Let's pretend we had to give a motel guest directions to their room. We would start by telling them the floor that they were on, and then which room on that floor was theirs. Multi-dimensional arrays work the same way, except the floors are numbered from 0. To make this a little clearer, consider this diagram below, which represents an array called letterArray:

|row/col|	0	|1	|2	|3	|4|
|-|-|-|-|-|-|
0|	y	| | |	w|
1|	| |	x| | |
2| |	z| | | |

We can see that the position of the letter w is row 0, column 4. We can write this position as letterArray[0][4]. The position of letter x is row 1, column 2, so we write that as letterArray[1][2]. Remember, row first, then column.

You can use a multidimensional array in exactly the same way as a one-dimensional array, and you can assign values the same way, too:

`letterArray[3][1]="a";`

### Task 7.3:

Determine the correct code to access the elements containing letters y and z in the diagram above.

Congratulations! You have finished the C++ tutorial!

Don't forget to turn your cups to red so that a member of the camp staff can check your code.

Once your project team is finished, one of the camp staff will bring you the materials for your project and explain how to get started. Until then, you can explore coding and the Linux environment, and you can let your other team members know that you are available if they have questions.
