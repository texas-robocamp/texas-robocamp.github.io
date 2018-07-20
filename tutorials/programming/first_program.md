---
title: "First Program: Hello World"
tags: [c++]
keywords: c++
last_updated: July 2, 2018
summary:
sidebar: tutorials
permalink: first_program.html
---


It's customary in the world of computer science that whenever you learn a new programming language, you always begin by writing a program to display the text Hello World to the screen. In programming, displaying text is referred to as "printing," so we're going to learn how to print "Hello World".

Whenever we begin a C++ program, we begin by writing a function called main. A function is a group of instructions about how to do some common task. The main function is the first function that executes when we run our C++ program, and it often uses other functions inside it. Using the cereal example above, imagine that we had a function called getMilk that contained instructions for how to get and open the milk from the fridge. Let's say we had functions for each part of the algorithm above. In our main function, we would "call", or ask the computer to run, each of those functions in the order that we wanted them to occur.

Sometimes, functions are already written for us and are stored in libraries. Using these functions saves us time and makes programming easier. For example, in C++ , you could write lines and lines of code to make the computer print a message to the screen. Or, you could use the iostream library that already has a function called cout, which is much simpler!
Ok, we're ready to write our first program! Following the instructions for compiling and running via the command line (we suggest you open this page in a new tab and leave it open for reference), copy the following program, save it, and run it. Then read on for a description of what each part of the program does. Note that the first two lines are very important, so you should copy them, too!

```cpp
#include <iostream>
using namespace std;

int main(){
     cout << "Hello World!\n";
}
```

Now we'll take a look at the code line-by-line. First, we tell the computer to include all the functions included in the iostream library when it compiles the program, so that we get to use cout. We always enclose library names in angle brackets. The next line, using namespace std; tells the compiler that library functions we use (like cout) are the ones included in the std namespace. For the most part, you just want to include that line in your code and not worry about why until some later class.

Next, we define our main function by saying int main(). This tells the computer that there is a function called main that returns an integer (more on this later). We then put an open curly brace to begin the code that is part of the main function.

We only have one command for this program. We tell the computer to use the cout function and then put what we want to print after <<. You can think of the double left-angle brackets as an arrow pointing toward cout, meaning that we are sending something (in this case, "Hello World\n ") to cout. We put quotation marks around the words we want to display (this is called a string), and end the string with "\n". This is a special character that tells the computer to go to the next line, kind of like hitting Enter at the end of a line when you are typing. In C++, every code line (ones without braces) ends in a semi-colon. So, we are telling the computer to send our string to cout to be displayed on the screen. We then use a closing brace to end the function, and with it, the program.

### Task 1:
Modify the Hello World program so that it prints, "Hello World and Your Name". For example, if your name is Justine, it would display, Hello World and Justine. Next, add a second line of text.

Yay! You have written your first C++ program!
Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Now, you'll learn how to perform [simple math](simple_math_user_input.html).
