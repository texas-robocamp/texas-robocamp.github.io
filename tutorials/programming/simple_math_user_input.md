---
title: "Simple Math and User Input"
tags: [c++]
keywords: c++
sidebar: tutorials
permalink: simple_math_user_input.html
---

Now let's move on to some simple math programs.

Let's start by writing a program that converts a person's weight on earth to their weight on the moon. An object that weighs 1.0 pounds on Earth would weigh 0.1654 pounds on the moon.

In C++, we represent multiplication with the * symbol.

This program, will ask the user of our program their weight, store that information, do the conversion, and then print their moon weight.

The easiest way to do this is to save all the numbers in the program as <b>variables</b>. Just like in math class, a variable is a letter or a word that stands for a number or a mathematical formula.

For example, we can say:

```cpp
x = 4;
y = 5 * x;
```

{{ site.data.alerts.tip }}
Unlike in math class, the `=` sign means <b>assignment</b>. It sets the value of the variable, but does not state that those two things are equal forever.

So if you write:
<br>
<br>
y = 0;
<br>
x = y;
<br>
y = 5;
<br>
<br>

Then at the end, `x` has `0` in it, NOT 5.
{{ site.data.alerts.end }}

{{ site.data.alerts.tip }}
When you name your variables, try to use better names than x and y. A good variable name describes the value that it stands for.
{{ site.data.alerts.end }}

{{ site.data.alerts.tip }}
Variables names can't have spaces in them and cannot start with numbers.
{{ site.data.alerts.end }}

## Types

C++ requires that we define the type of our variables.

For example, if we know that a variable could store a number with a decimal, it should be a float, which stands for floating (decimal) point number.

We always declare our variables before they are used. Generally we do this by saying the variable type and then the names of all the variables that are that type at the first line of the function.

Here's a quick reference of types in C++:

Type    | Example
--------|-------------
float	| 0.124 or 4.0
int	    | 4 or 134
string	| "Hello World"
bool	| true/false

Let's write an algorithm for this program, so that we make sure the computer has all the information it needs to do the program.

1. Save the conversion factor (0.1654) as the variable `conversionFactor`

2. Ask the user's weight

3. Save the weight to a variable called `earthWeight`

4. Multiply earthWeight and conversionFactor and save the result as the variable `moonWeight`

5. Display `moonWeight` to the user

## Getting User Input

This program will require input from the user. To get that input, we will use `cin`.

`cin` takes whatever the user types and assigns it to a variable. The <b>syntax</b> (the structure) of the command looks like this:

```cpp
cin >> myVariable;
```

{{ site.data.alerts.tip }}
Note the similarity to `cout`.
{{ site.data.alerts.end }}

When the computer reaches `cin` in the program, it will display a prompt for the user to type. The user can then type and hit the Enter key, and the computer saves whatever the user typed to the variable called myVariable.

## Printing a Variable

The last thing we have to address is how to print out the person's weight on the moon after we do the conversion. We do this by using a second `<<`, just like before `endl` in the previous tutorial.

```
cout << "You would weigh "<< moonWeight << " lbs on the moon." << endl;
```

This puts together the two strings and the variable moonWeight and sends the whole thing to cout to be displayed.

## Exercise 3.2.1: Full Program

Now we're ready to write the program.

Any text preceded by `//` is a comment.

Comments are ignored by the compiler and are used by programmers to explain parts of their code.

{{ site.data.alerts.tip }}
It's good practice to have short comments in your code, so you can remember what that code snippet does when you return to it later.
{{ site.data.alerts.end }}

```cpp
#include <iostream>
using namespace std;

int main(){
  float conversionFactor, earthWeight, moonWeight;  //define variables as floats

  //Prompt the user to enter weight
  //Note that there is no "endl", so the prompt will appear on the same line.
  cout << "Enter your weight on earth:";
  cin >> earthWeight; //Store what the user types as earthWeight
  conversionFactor = 0.1654;
  moonWeight = earthWeight*conversionFactor;
  cout << "You would weigh " << moonWeight << " lbs on the moon." << endl;  //print out conversion
}
```

## Operators

There are lots of math operators available in C++. The most common are the following:

Operator |	Operation
:-------:|:---------
+	     | addition
−	     | subtraction
*	     | multiplication
/	     | division
%	     | modulus (remainder)

When you divide two integers, the remainder of the two numbers is left out. This is called <b>truncation</b> and the value was <b>truncated</b>.

In integer division., `5 / 2` will give you 2, not 2.5. If you want the full value, you must use floats.

The `%` (<b>modulus</b>) operator gives you the remainder of integer division.

So if you want the remainder of `5 / 2` (which is 1), you would say:

```cpp
int rem;
rem = 5%2;
```

This is very useful in programming, since you can easily decide is a number is even, odd, or a multiple of some other number.

## Exercise 3.2.2:

Write a program that reverses the program above. This program will take the user's weight on the moon and convert it to Earth weight.

{{ site.data.alerts.tip }}
What's the mathematical opposite of multiplication?
{{ site.data.alerts.end }}

## Exercise 3.2.3:

Write a program that asks the user for two numbers, and then prints out the sum of the two numbers. Hint: Use multiple cin commands.

{% include callout_red_cup.html task="[Tutorial 3.2]" %}

{% include note.html content="Campers should demonstrate the output of Exercise 3.2.1, Exercise 3.2.2, Exercise 3.2.3 to camp staff." %}

Next, you'll learn about [functions](functions.html).
