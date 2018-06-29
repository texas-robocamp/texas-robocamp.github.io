## Introduction to C++ Programming

This tutorial is aimed at preparing you with a working knowledge of C++ programming, how to operate the Linux machines, and how to compile and run your programs via the command line.

Below is a complete list of the topics we will be covering. You have a list of tasks corresponding to these topics, which the lab staff will sign off on as you complete them. Don't forget to switch off when pair programming and keep track of who's driving! You can save this log anywhere in your home directory. Once you have completed all tasks in this tutorial, you are free to move on to programming your lights!

## Section 1: Welcome to C++!
In this tutorial, we'll cover the basics of C++ programming so that you'll have all the information you need to complete the project. It's designed for the complete programming novice, so if you're an experienced programmer, skim the following and then feel free to jump right into the example programs!
C++ is a programming language that you can use to control your computer. A programming language is just like a regular language---it has a vocabulary and a set of rules for how to make commands. A program is nothing more than a list of commands, or instructions, that you give to the computer. When we write a program, we first begin by thinking very carefully about what we want the computer to do, and create a step-by-step plan for how the computer will do it. This step-by-step plan is called an algorithm. For example, an algorithm for making a bowl of cereal might look like the following:

- Take out a bowl from the cabinet and place it on the counter
- Take out a box of cereal from the cabinet and open it
- Pour the cereal from the box into the bowl until it almost reaches the top of the bowl
- Close the box and put it back in the cabinet
- Open the fridge and take out the milk
- Open the milk and pour it into the bowl until the cereal reaches the top of the bowl
- Close the milk and put it back in the fridge
- Take a spoon from the silverware drawer and put it in the bowl

Notice how detailed that algorithm is! When you write an algorithm, you want to be as detailed and specific as possible, because computers are really stupid. They only know as much as you tell them, so if you want to the computer to do something correctly, you'd better tell it exactly how and give it enough information to complete the task.

We'll come back to the idea of an algorithm in just a moment, but first let's write our first C++ program.

## Section 2: First Program: Hello World

It's customary in the world of computer science that whenever you learn a new programming language, you always begin by writing a program to display the text Hello World to the screen. In programming, displaying text is referred to as "printing," so we're going to learn how to print "Hello World".

Whenever we begin a C++ program, we begin by writing a function called main. A function is a group of instructions about how to do some common task. The main function is the first function that executes when we run our C++ program, and it often uses other functions inside it. Using the cereal example above, imagine that we had a function called getMilk that contained instructions for how to get and open the milk from the fridge. Let's say we had functions for each part of the algorithm above. In our main function, we would "call", or ask the computer to run, each of those functions in the order that we wanted them to occur.

Sometimes, functions are already written for us and are stored in libraries. Using these functions saves us time and makes programming easier. For example, in C++ , you could write lines and lines of code to make the computer print a message to the screen. Or, you could use the iostream library that already has a function called cout, which is much simpler!
Ok, we're ready to write our first program! Following the instructions for compiling and running via the command line (we suggest you open this page in a new tab and leave it open for reference), copy the following program, save it, and run it. Then read on for a description of what each part of the program does. Note that the first two lines are very important, so you should copy them, too!

```#include <iostream>
using namespace std;

int main(){
     cout << "Hello World!\n";
}
```

Now we'll take a look at the code line-by-line. First, we tell the computer to include all the functions included in the iostream library when it compiles the program, so that we get to use cout. We always enclose library names in angle brackets. The next line, using namespace std; tells the compiler that library functions we use (like cout) are the ones included in the std namespace. For the most part, you just want to include that line in your code and not worry about why until some later class.

Next, we define our main function by saying int main(). This tells the computer that there is a function called main that returns an integer (more on this later). We then put an open curly brace to begin the code that is part of the main function.

We only have one command for this program. We tell the computer to use the cout function and then put what we want to print after <<. You can think of the double left-angle brackets as an arrow pointing toward cout, meaning that we are sending something (in this case, "Hello World\n ") to cout. We put quotation marks around the words we want to display (this is called a string), and end the string with "\n". This is a special character that tells the computer to go to the next line, kind of like hitting Enter at the end of a line when you are typing. In C++, every code line (ones without braces) ends in a semi-colon. So, we are telling the computer to send our string to cout to be displayed on the screen. We then use a closing brace to end the function, and with it, the program.

### Task 2.1:  
Modify the Hello World program so that it prints, "Hello World and Your Name". For example, if your name is Justine, it would display, Hello World and Justine. Next, add a second line of text.

Yay! You have written your first C++ program!
Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Now, you'll learn how to perform simple math.


## Section 3: Simple Math and User Input
Now let's move on to some simple math programs. Let's start by writing a program that converts a person's weight on earth to his or her weight on the moon. An object that weighs one pound on Earth would weigh 0.1654 pounds on the moon, according to Wikipedia. Therefore, if we know the weight of something on Earth, we can multiply that weight by 0.1654 to get its weight on the moon. In C++, we represent multiplication with the * symbol.

In order to write this program, we will need to ask the user of our program for his or her weight, store that information in the computer, do the conversion, and then display the moon weight back to the user. The easiest way to do this is to save all the numbers in the program as variables. Just like in math class, a variable is a letter or a word that stands for a number or a mathematical formula. For example, we can say
```x = 4;
y = 5 * x;
```
and whenever we use the variable x in our program, the computer will know that the x means 4. Likewise, the computer knows that y means 20. When you write variables, try to use better names than x and y. A good variable name describes the value that it stands for. So, if we have a variable that stands for a person's weight on earth, we should call that variable earthWeight. Keep in mind that variables can't have spaces and should not start with numbers.
In C++ , we are required to define the type of our variables. For example, if we know that a variable will store a decimal point number, we declare it to be a float, which stands for floating (decimal) point number. We always declare our variables inside a function before writing any other code (so for the main function, it would be the first line under int main(){). We do this by saying the variable type and then the names of all the variables that are that type, followed by a semi-colon. For example, we would say float earthWeight for the variable earthWeight described above. For our program, all our variables will be floats, since they are all decimal point numbers. Here's a quick reference of types in C++:

Type |	Example|
|:---:|:---:|
|float	|0.124 or 4.0|
|int	|4 or 134|
|string	|"Hello World"|
|bool	|true/false|

Let's write out an algorithm for this program, so that we make sure the computer has all the information it needs to do the program.

1. Save the conversion factor (0.1654) as the variable `conversionFactor`

2. Ask the user's weight

3. Save the weight to a variable called `earthWeight`

4. Multiply earthWeight and conversionFactor and save the result as the variable `moonWeight`

5. Display `moonWeight` to the user

The only tricky part of this program will be getting input from the user. To do that, we will need to use a special command called cin. This command takes whatever the user types and saves it to a variable. The syntax (the structure) of the command looks like this:

`cin >> myVariable;`

Notice that the structure of cin is analogous to cout, which we learned about in the last lesson! When the computer reaches the cin command in the program, it will display a prompt for the user to type. The user can then type and hit the Enter key, and the computer saves whatever the user typed to the variable called myVariable. In our program, we will replace myVariable with earthWeight.
The last thing we have to address is how to print out the person's weight on the moon after we do the conversion. We can use cout again and simply surround the variable with double left-angle brackets, like this:

`cout << "You would weigh "<< moonWeight << " lbs on the moon.\n";`

This puts together the two strings and the variable moonWeight and sends the whole thing to cout to be displayed.
Now we're ready to write the program. The text preceded by "//" is a comment. Comments are ignored by the computer and are just used by programmers to explain parts of their code. It's good practice to have short comments in your code, so you can remember what that code snippet does when you return to it later.

```#include <iostream>
using namespace std;

int main(){
     float conversionFactor, earthWeight, moonWeight;  //define variables as floats

     cout << "Enter your weight on earth:";  //prompt the user to enter weight
     cin >> earthWeight; //store what the user types as earthWeight
     conversionFactor = 0.1654;  
     moonWeight = earthWeight*conversionFactor;
     cout << "You would weigh " << moonWeight << " lbs on the moon.\n";  //print out conversion
}
```

There are lots of math operators available in C++. The most common are the following:

| Operator |	Operation |
| -------|----------|
|+	       |addition    |
|−	 |subtraction|
|*	|multiplication|
|/	|division|
|%	|modulus (remainder)|

The % operator gives you the remainder of integer division. So if you want the remainder of 5/2 (which is 1), you would say:

rem = 5%2;
This is very useful in programming, since you can easily decide is a number is even, odd, or a multiple of some other number.

### Task 3.1:
Write a program that reverses the program above. This program will take the user's weight on the moon and convert it to Earth weight. Hint: What's the mathematical opposite of multiplication?

Good job! Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
### Task 3.2:
Write a program that asks the user for two numbers, and then prints out the sum of the two numbers. Hint: Use multiple cin commands.

Woot, woot! Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Next, you'll learn about functions.

## Section 4: Functions

Functions are a way to organize chunks of your code together. A function is a group of statements about a related subtask that is bundled together by a name. For example, we might have a getLetter function that asks the user for a lowercase letter, or an averageNumbers function that averages a list of numbers. Functions are important because they allow your code to be modular---you write a function once, and you can use that function over and over.

We have already seen some functions. main() is the most important function in a program, because it contains all the code necessary to start the program and is responsible for calling other functions. Let's take a second to review the setup of main:

```
int main(){
     //code goes here
}
```

We begin by declaring the return type of main to be an integer. This tells the computer that we intend for main to send back an integer when it is finished. Usually, this is accomplished through a return statement, and, by convention, main returns a 1 if there were an error and 0 if the program completes successfully. We haven't been including a return statement and, for main, it isn't absolutely necessary. We'll talk more about return types for other functions soon.

We then write the name of the function, main, and follow it with a set of parentheses. These parentheses are for parameters. Parameters are things like numbers or variables that are sent into a function so that it can use them in its code. For our purposes, they are empty for main, but we'll see them in action in other functions soon.

Finally we have a set of curly braces, and we put the code that belongs to main inside the curly braces.

Let's try writing a very simple function that prints the greeting "Hello World!". We will call (run) this function from main. It won't take any parameters or return anything, so the return type of this function will be void. Here's how we would set it up:

```#include <iostream>
using namespace std;

void printGreeting(){
   cout << "Hello World!\n";
}

int main(){
   printGreeting();
}
```

Here, we made the printGreeting function exactly like we made main. We declared its return type to be void, named it printGreeting, and put no parameters inside the parentheses. Inside the curly braces, we wrote the same code we used for our very first C++ program.

What's interesting is how we call, or run, the function. Inside of main, we call the function by naming it and then following it by a set of empty parentheses, which we'll explain in a minute. As always, we end the line with a semi-colon.

The way that this program works is the following: the computer begins running the program inside the main function. When it gets to the printGreeting function, it "jumps" to where we defined the printGreeting function and starts executing that code. When it gets to the end of the printGreeting function, it jumps back to the main function exactly where it left off and continues executing the rest of the code in main.

### Task 4.1:
Type in the code above and see that it executes. Modify it so that main calls printGreeting() twice. How does the output change?

Good work! Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Let's look at a slightly more complicated function that uses parameters. Parameters allow you to customize a function by performing some set of tasks over different values. In this example, we're going to change printGreeting so that it prints out "Hello, name", where name is some name that the user enters. printGreeting will be sent the name to print out from main. So, the general layout of this program will be: start in main, get a name from the user, send that name to printGreeting, print out the hello message, go back to main. It sounds complicated, but we think you'll master it.

When a function takes a parameter, we need to tell the computer what kind of parameter to expect. So, since we want to give printGreeting a name, the parameter will be a string. We also want to give the computer the name of the parameter to expect, which we can use inside the printGreeting function (but only inside this function). So we will call it name. So, the printGreeting function will look like this, with the name variable used in the cout function:

```
void printGreeting(string name){
   cout << "Hello " << name << "!\n";
}
```

Now the computer knows that anything that calls the printGreeting function will need to provide a string value that will be called name inside the function. So, in order to call the function, we need to get some input from the user and provide that input when we call the printGreeting function from main:

```
int main(){
   string userName;
   cout << "Enter a name: ";
   cin >> userName;
   printGreeting(userName);
}
```

Here, we got a string userName from the user and sent it to the printGreeting function by placing it inside the parentheses.
Before we go any further, we need to have a discussion about the difference between arguments and parameters. The value that is placed in the parentheses has a special name: it's the argument to the function. (Or, if there are more than one, they are the arguments!) Arguments differ from parameters in that arguments are what you send to a function and parameters are what a function expects to receive. When the printGreeting function is called from main, the argument userName's value is copied to the parameter name in printGreeting. While the computer is executing the code in printGreeting, it can't use the variable userName, because that variable is only defined (available) in main. So that's why we have a parameter name—the value of userName gets copied to name so that the program has access to that data. When the computer finished executing printGreeting and returns to main, it can again use the variable userName, but it can no longer use name. This is what is known as the scope of a variable, which means where the variable is defined and can be used.

As we mentioned before, on of the reasons that functions are so great is that we can reuse them and give them different arguments. For example, without changing the printGreeting function at all, we can use it to print greetings to three different people:

```#include <iostream>
using namespace std;

void printGreeting(string name){
   cout << "Hello " << name << "!\n";
}


int main(){
   printGreeting("Alison");
   printGreeting("Brian");
   printGreeting("Clifford the Big Red Dog");
}
```
And we would get the following output:
```
Hello Alison!
Hello Brian!
Hello Clifford the Big Red Dog!
That's pretty convenient.
```

### Task 4.2:
Using this code as a base, add two calls to printGreeting() in main so that both of you are also greeted.
Now that you've been greeted, don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
We can also use functions to do computation for us and return the answers back to main. For example, let's imagine that we want to write a function called squareANum that calculates the square of a number. We know from math class that the square of a number is that number multiplied by itself. So, let's write a function that takes a floating point number and squares it. After we figure out the square of the number (which we'll call numSquared), we'll need a way to return our answer back to the function that called it. To do this, we simply say:

`return numSquared;`

This is called the return statement, and it is always the last line executed in a function. Since we know that we are squaring floats, we will be returning a float, so the return type of this function will be a float as well. Now we're ready to write our function:

```float squareANum(float num){
   float numSquared;

   numSquared = num * num;
   return numSquared;
}
```

We write the function as we normally would, and then tell the computer to return numSquared to the function that called it. Since we are returning something to main, we need to set up a variable to store the value that we are returning. This part of programming is like playing a game of catch. You can imagine that squareANum and main are playing a game of catch, and squareANum is throwing a ball named numSquared at main. If main doesn't catch the ball, the ball will fall in the grass and be lost. In programming, main catches the ball by having a variable in place to store the return value---if it doesn't, the value is gone forever. Since we want to store the result, we'll set our call to squareANum in main equal to the variable result. Here's the whole program:

```#include <iostream>
using namespace std;

float squareANum(float num){
   float numSquared;

   numSquared = num * num;
   return numSquared;
}

int main(){
   float numToSquare, theSquaredNumber;

   numToSquare = 5.5;
   theSquaredNumber = squareANum(numToSquare);  //catch the returned value
   cout << numToSquare << " squared equals " << theSquaredNumber << ".\n";
}
```
Note that you must either a) write a function's code earlier in the program text than it is called or b) include a prototype of that function at the top of the file. A function prototype is the return type, name, and parameter definition followed by a semicolon. For instance, the prototype for squareANum would be:
float squareANum(float num);
Function prototypes usually appear before the first function and after the include files.

### Task 4.3:
Write a program similar to the one above that triples a number and adds 5 to it.
Yay! Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Finally, we can write functions that take multiple parameters---and those parameters can even have different types. For example, we can modify the printGreeting function so that it takes both a string name and an integer age, and prints out a message about that person's age:

```#include <iostream>
#include <string>

using namespace std;

void printGreeting(string name, int age){
   cout << "Hello " << name << "!\n";
   cout << "You are " << age << " years old.\n";
}


int main(){
   printGreeting("Ronald McDonald", 49);
}
```

We just have to remember to send the function all parameters it expects as arguments when we call it. Although we can have different numbers and types of parameters, a function can only return one value, so think carefully about what result you want from a function before you write it.

### Task 4.4:
Modify the printGreeting function to also take a string birthdayMonth parameter, and print out a message that tells them how old they will be the next time it is that month. For the Ronald McDonald example, the output might be, "You will be 50 next January".

You've done a great a job learning about functions. Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Now, let's learn about a special data representation, the object.



## Section 5: Building Objects
Sometimes we want to group different kinds of data together. For example, let's think about a bank account. What kind of information does your bank account store about you? Most important are your bank account number (an integer) and your account balance (a float). It also stores your name and address (both strings). Often in programming, we want to group related data like this together in a single construct, and we can do that with objects.

A class defines an object type, the variables inside the object (such as bank account number and balance), and functions you can perform on the object (such as deposit() and withdraw()). So we begin by defining a class.

The first step is easy enough: type the keyword class, and then choose a class name beginning with an uppercase letter (this is not required, but is the convention followed by programmers). For our bank account example, we'll name the class BankAccount:
```
class BankAccount { 

};
```

You'll notice the class definition will be encompassed by curly braces and has a trailing semi-colon. Now we need to add some variables and functions.

Variables inside a class are called member variables, so the member variables for this class are the account number, account balance, your name, and your address. Adding these to our skeleton definition we get the following:
```
class BankAccount {
  public:
     int accountNumber;
     float accountBalance;
     string name;
     string address;
};
```

The public keyword indicates that the member variables (and functions—we'll add those later) that follow are available to the rest of the program to use. You can also declare a class's variables and functions to be private, but we'll cover that later. Notice, too, that the variables are declared in the same way as before, indicating the type and giving it a descriptive name.

In order to create a BankAccount object, we first need to define a constructor. The constructor is a piece of code that executes when a new object of this class is created. Inside the constructor, we want to set initial values for any member variables we have, otherwise our program may not work as we intend. A constructor is defined in the the same way as a function; the only differences are: 1) the name of the constructor must match the name of the class and 2) the constructor has no return type.

For a brand new BankAccount, when we don't yet know anything about the data that will be put into it, default values of zeroes and empty strings will be fine; we can always set them to something else later. Thus, our BankAccount constructor is defined as follows:
```
class BankAccount {
  public:
      int accountNumber;
      float accountBalance;
      string name;
      string address;

      BankAccount() {
          accountNumber = 0;
          accountBalance = 0.0;
          name = "";
          address = "";
      }
};
```
We have now defined a basic data structure that holds bank account information, and so we can now write a program that creates a BankAccount object and sets the values of its fields. Notice the class must be declared before the main function where we create our object.

```
#include <iostream>
using namespace std;

class BankAccount {
  public:
     int accountNumber;
     float accountBalance;
     string name;
     string address;

      BankAccount() {
          accountNumber = 0;
          accountBalance = 0.0;
          name = "";
          address = "";
      }
};


int main() {
     BankAccount alisonBankAccount; //make a bankAccount for Alison
     alisonBankAccount.accountNumber = 14538;  //assign the account a number
     alisonBankAccount.accountBalance = 1000000.01;  //give Alison some money
     alisonBankAccount.name = "Alison Norman";  //assign name and address
     alisonBankAccount.address = "1600 Pennsylvania Avenue NW Washington, DC 20500";
}
```
Look at the first line in main. We've created a BankAccount object called alisonBankAccount, and it has been initialized using the default constructor.

We use dot notation to access and set the public variables of an object, as seen in the remaining lines in main. In dot notation, we access member variables by first writing the name of the object (e.g., alisonBankAccount), then a dot, and then the name of the field we want (e.g., accountNumber). From here, we see that we can create multiple different BankAccounts and populate them all with different balances and information. For example, let's say that in addition to Alison's bank account, we'll make an account for Bob Bobson. Here's the same code as above, but now we've added a second instance of the BankAccount class:
```
//make two different bank accounts
BankAccount alisonBankAccount;
BankAccount bobBankAccount;

//assign two different account numbers
alisonBankAccount.accountNumber = 14538;  //assign the account a number
bobBankAccount.accountNumber = 54324;

//assign different account balances
alisonBankAccount.accountBalance = 1000000.01;  
bobBankAccount.accountBalance = 2.52;

//give Bob some money to get out of debt
//i.e. add 300.95 to the current value of accountBalance
bobBankAccount.accountBalance += 300.95; 

cout << "Bob has $" << bobBankAccount.accountBalance << ".  Yay Bob!\n";
```
We can give Alison and Bob's BankAccount objects completely different information, and we can also modify the values that the fields hold. Notice how Bob made a timely deposit with the line:

`bobBankAccount.accountBalance += 300.95;`

The += operator means that you increase the value of the variable to the left of the += by the value on the right side of the += and save it as the same variable. In other words, the following two lines are equivalent:

`bobBankAccount.accountBalance += 300.95;`

`bobBankAccount.accountBalance = bobBankAccount.accountBalance + 300.95;`

These objects do a great job of cleaning up our code—consider having to keep track of Alison and Bob's bank account info without them and what happens when we want to add a third bank account—all those variables add up!

### Task 5.1:
Using this code as a base, add some money to Alison's account and then output her balance.
Alison says, "Thank you!" Before moving on, don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
In the program above, we declared the member variables as public, but, as mentioned before, another option is private. Private member variables can only be accessed by other objects of their type—in our case, our BankAccountobjects. There are a number of reasons to declare member variables as private, one of which is to maintain quality control on their values—so that account balances can't be set to negative, for instance. In general, you will want to declare variables and functions necessary for the inner workings of the class private and make public the variables and functions necessary for object to be useful to the rest of the program.

Consider the usefulness of using public functions to modify private data, as opposed to leaving the data public. What if we wanted to apply the same set of checks to make sure the account balance is always updated properly—for example, checking for overdrafts or account deposit limits. With our data safely hidden behind the private wall and the use of functions, we can say with confidence our bank account balances are correct.

Returning to our BankAccount class, let's go ahead and declare our member variables as private, as shown below.

```
class BankAccount {
  public:
      BankAccount() {
          accountNumber = 0;
          accountBalance = 0.0;
          name = "";
          address = "";
      }
  private:
     int accountNumber;
     float accountBalance;
     string name;
     string address;
};
```
Now that these variables are private, our object is actually fairly useless right now. We left the constructor public, so we can create new objects, but all our data is inaccessible from outside the class because it's declared private. To make our object useful, we'll introduce member functions, which define actions that can be performed on or with the member variables.

Consider what behavior a BankAccount should exhibit. What sorts of actions are necessary? [Think a minute and then read on!]

People need to be able to deposit to and withdraw from their accounts, as well as see their current account balance. The latter we can solve with a simple getAccountBalance function that just returns the value stored in accountBalance. The deposit and withdraw operations are a little more complex; both modify the account balance by a given amount. What might the deposit function look like? Would it take any arguments? What kind? Does it need to return anything? [Again, think a minute before continuing!]
deposit() would need a float as an argument, which is the amount being deposited, and then would need to update the accountBalance variable appropriately. It doesn't need to return anything. Since this is a member function, we write its instructions inside the class definition, like this:

```
#include <iostream>
using namespace std;

class BankAccount {
  public:
     float getAccountBalance() {
          return accountBalance;
     }

     void deposit(float amount) {
          accountBalance += amount;
     }
      BankAccount() {
          accountNumber = 0;
          accountBalance = 0.0;
          name = "";
          address = "";
      }
  private:
     int accountNumber;
     float accountBalance;
     string name;
     string address;
};
```
Everything under the public keyword and above the private keyword is our class's public members, and everything below the private keyword is the class's private members. We want both functions to be public; they are the rest of the program's interface with our BankAccount data.
Now when we want to deposit money in main, we won't need to type:

 `bobBankAccount.accountBalance += 300.95;`
Instead, we'll be able to say:
 `bobBankAccount.deposit(300.95);`
Thus, our modified main looks like this:
```
int main(){

   BankAccount alisonBankAccount;
   BankAccount bobBankAccount;
   bobBankAccount.deposit(300.95);
}
```

There are a few new things to note.
The syntax for defining a member function is the same as any other function, only it goes inside the class definition.
Inside our member functions we have access to our member variables, as if we had declared them locally inside the function.
Outside of the class definition, we use the dot notation to call an object's member functions; in this case, we have our bobBankAccount object, followed by a period, followed by the function call. Just as with the function calls you've made previously, you start with the function name, followed by any arguments wrapped in parentheses.
Task 5.2:
Add another bank account to the example above. Deposit $500 dollars into Alison's account, then withdraw $400 from Bob's account and place it in the account you created. Then print out the balances from all three accounts. (You'll need to write the withdraw function. How is it similar to the deposit function? How is it different? What happens if you attempt to withdraw an amount greater than your balance?)

Good job!
Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!

### Task 5.3:
Create a superhero class. The fields should include the superhero's name, arch-nemesis, weakness, and sidekick (these are all strings). You are welcome to add some extra fields of your own. Create a default constructor, some methods to set the fields, and a method to print the information in the fields. Then make some instances of the superhero class using your favorite superheroes for inspiration, set the fields appropriately, and print their information.

Congratulations, you are well on your way to mastering the complex subject that is object-oriented programming!
Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Now, let's move on to decisions.

## Section 6: Decisions

In this section, we're going to learn how to control the flow of a C++ program. Sometimes, you want your program to do one thing if certain conditions are met and to do another thing if other conditions are true. We make decisions and control the flow of the program using logical expressions and if-else statements.

Let's imagine we have a program that helps police officers in a town give speeding tickets to drivers based on how fast the driver was going. In this imaginary town, the speed limit is 40 mph, but there are extra penalties if a person goes over 60 mph.

Here's how the program will work: the policeman (the user) enters the speed of a vehicle in the program. If the vehicle was going over 40 mph, the driver gets a $100 speeding ticket. But, if the vehicle was going over 60 mph, the driver gets charged $250. If the vehicle was going 40 mph or under, the driver doesn't get a ticket. The program will display the fine to the user.

In order to do this, we will need to test the speed that the user types in (we'll save this to a variable called driverSpeed and see if it is greater than 40, and then see if it is also greater than 60. Just like we would in math, we can write this expression using the greater than/less than symbols and store it in a variable:

`isSpeeding = driverSpeed > 40;`

In programming languages, the above statement can either be true or false, depending upon the value of driverSpeed, so isSpeeding will either be true or false. A variable that is either true or false is called a boolean. In C++ , a boolean that is true displays as a 1, and a boolean that is false displays as a 0 when you print it out to the screen. So, in our line of code above, if the value of the variable driverSpeed is 20, and you printed out isSpeeding, you would see a 0, because 20 is not greater than 40 and thus the statement is false.

The following chart shows all the logical operators, most of which should be familiar to you from math class:

|Logical Operator	|Meaning	|Example of True Statement|
|---:|:---:|:---:|
|>	|greater than|	5 > 4|
|>=	|greater than or equal to|	5 >= 5|
|<	|less than	|2 < 5|
|<=	|less than or equal to|	2 <= 3|
|==	|is exactly the same as|	3 == 3|
|!=	|is not the same as	|3 != 5|

The last two operations, == and !=, are called equivalency operators because they check if two things are exactly the same, or equivalent. For example, 9==9 is true because 9 is exactly the same as 9. Additionally, if we have a variable myVar and is set to 9, then myVar==9 is true. However, 9=="nine" is false because an integer (9) is never exactly the same as a string ("nine"). In general in C++, you cannot use logical operators to compare strings at all.

To test values and perform a set of instructions (or not) based on the result, we use if statements, which are used in programming exactly the same way we use it in real life. "If" something is true, we do one thing; "else" (otherwise) we do something else. To come back to the speeding ticket program we described above, we could test whether the variable driverSpeed is greater than 40 by using the following syntax:
```
if (driverSpeed > 40){
   ticketPrice = 100;
}
else{
   ticketPrice = 0;
}
```
When we use if statements, we put the conditional clause, which is the logical expression we are checking or testing, in parentheses after the word "if". Then we put all of the code that we want the computer to execute if the conditional clause is true inside of curly braces. After an if, we can have: an else if, which works exactly the same as an if but is only checked by the computer when the if statement is false; an else, which is the code that is executed if the if statement is false, or nothing.

Please ask questions if any of this explanation didn't make sense—we are happy to explain in person. (Just turn your cups to red!)

We now know everything we need to be able to write the speeding ticket program. We will use an if-else pattern for this program, so we will first check if the speed is greater than 60, and if that is false we will check if the speed is greater than 40, and if that is also false, then we will give a $0 speeding ticket.
```
#include <iostream>
using namespace std;

int main(){

   int driverSpeed, ticketPrice;
   
   cout << "Hello policeman.  Enter the speed of the vehicle:";
   cin >> driverSpeed;
   
   if (driverSpeed > 60){        //First check if driver is excessively speeding
      ticketPrice = 250;
   }
   else if (driverSpeed > 40) {  //Then check if driver was going over 40mph
      ticketPrice = 100;
   }
   else{                         //If neither of those were true, the driver wasn't speeding
      ticketPrice = 0;
   }
   cout << "The ticket is $" << ticketPrice << ".\n";
}
```
### Task 6.1:
Modify the above code so that it prints out a separate message for each different level of ticket. For example, if the driver was going over 60 mph, the program could print "You should severely punish the driver for endangering people!" and for a non-speeding ticket, the program could print, "Congratulate the driver on maintaining a safe speed."

Thanks for enabling better communication. Now, don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
You can also nest if-statements inside each other. For example, let's say that there is an additional law about wearing seat belts in our imaginary town. If you are speeding and you are also not wearing a seat belt, you get fined an additional $55. We can add this into our code like this:
```
   [...] // ignoring the beginning stuff

   int driverSpeed, ticketPrice;
   string wearingSeatbelt;     //string variable

   //get speed
   cout << "Hello policeman.  Enter the speed of the vehicle:";  
   cin >> driverSpeed;

   //user will type y or n
   cout << "Was the driver wearing a seat belt? y or n:";
   cin >> wearingSeatbelt;

   //First check if driver is excessively speeding
   if (driverSpeed > 60){
      //set the initial ticket price to 250
      ticketPrice = 250;

      //if the speed is over 60 AND
      //the driver wasn't wearing a seat belt
      if (wearingSeatbelt == "n"){
         //add 55 to the old ticket price (250+55=305)
         ticketPrice = ticketPrice + 55;
      }
   }
   [...]  //rest of the program
```
You can also check if two or more conditions are both true in the same if-statement. You do this by using &&. So you can say,
//true if both conditionals are true
```
if ((driverSpeed > 60) && (wearingSeatbelt == "n")){
   ticketPrice = 305;  //250 + 55
}
```
If BOTH the first clause (driverSpeed > 60) and the second clause (wearingSeatbelt == "n") are true, then the whole conditional is true. If either are false, the whole thing is false. There is also an "or" operator (||), which is true if either or both of the conditions are true:
```
// true if one or both conditionals are true
if ((driverSpeed > 60) || (wearingSeatbelt == "n")){
   cout<<"You broke the law, buddy!\n";
}
```
Note how we use parentheses to group each conditional, and then group both the conditionals into one statement.
Finally, there is the not operator (!), which negates a conditional clause (again, note the parentheses!):

```
if (!(wearingSeatbelt == "n")){
   cout << "The driver was wearing a seatbelt!\n";
}
```

### Task 6.2:
Finish implementing the seat belt law for the driver who goes over 40mph (just follow the format above). Then come up with another law and add it to your code. Be as creative and ridiculous as you want, and if you need help, ask for it!

Now, we'll move on to loops.
Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!


## Section 7: Repetition
Sometimes we want our code to execute multiple times. It would be really unfortunate if we had to keep typing the same exact lines of code again and again—what if we didn't know exactly how many times we wanted the code to repeat? Fortunately, we have loops to help us out. A loop is a section of code that the computer runs over and over again until it reaches some stopping criteria. There are two kinds of loops that we will work with: for loops and while loops.

A for loop repeats code for a specific number of times. If you know exactly how many times you want some lines of code to repeat, you'll want to use this type of loop. For example, let's say we want to print "Hello World" five times. We could write the following for loop:
```
for (int i=0; i<5; i++){
   cout << "Hello World\n";
   //more lines of code can go here
}
```
This will print:
```
Hello World
Hello World
Hello World
Hello World
Hello World
```
Let's dissect this code. We begin with the word "for", which tells the computer that we have a for loop. Then, inside the parentheses, we define how many times we want the loop to run, or we "initialize" the loop. The first phrase, int i=0;, defines the loop variable, which is the variable whose value dictates how many times the loop executes. We tell the computer that the loop variable will be an integer named i, and that it will begin by being equal to 0. The second phrase, i < 5;, is a logical expression that works just like those we saw in the last section—when used in loops, the computer will keep executing the loop as long as the expression is true. In this example, the computer will keep looping as long as i is less than 5; when i is greater than or equal to five, it will stop. The third phrase, i++, tells the computer how to modify the loop variable. The shorthand i++ means that i will increase by 1 every time the loop runs. Then, inside curly braces, we put all the code that we want to execute each time the loop runs.
Now we will look at another example. Say we want to count from 0 to 9, and we want to print out the number each time. We can easily use a loop to do this:
```
for (int i=0; i<10; i++){
   cout << "The current number is " << i << ".\n";
}
```
This will print:
```
The current number is 0.
The current number is 1.
The current number is 2.
The current number is 3.
The current number is 4.
The current number is 5.
The current number is 6.
The current number is 7.
The current number is 8.
The current number is 9.
```
Notice how 10 doesn't print, because we told the loop to stop as soon when i was no longer less than 10. The first time the loop executes, i equals 0, so the loop prints out the message "The current number is 0." and then adds one to i, making i 1. Then the loop executes again, and we print "The current number is 1." Then i is increased by 1 again, making it 2, etc.
We can also modify the loop so that the loop variable changes in different ways---just by changing the initialization and the modification pieces. Say we want to count backwards from 4 to 1. We can create the loop like this:
```
for (int i=4; i>=1; i--){
   cout << "The current number is " << i << ".\n";
}
```
Here, we begin with i set to 4. We loop while i > = 1. We tell the computer to subtract one from i each time the loop runs with i--. This loop prints:
```
The current number is 4.
The current number is 3.
The current number is 2.
The current number is 1.
```
We can also count by numbers other than one and negative one. Let's say we only want to print out odd numbers. We could change the loop to count by 2s from 1 to 7 in the following way:
```
for (int i=1; i<9; i+=2){
   cout << "The current number is " << i << ".\n";
}
```
With this loop initialization, we get the following output:
```
The current number is 1.
The current number is 3.
The current number is 5.
The current number is 7.
```
The loop begins with i=1 and increases by 2 every time. Notice that it stops before reaching 9. To make more complicated loops, all you have to do is think carefully about the loop counter. Where do you want it to start? Where should it stop? How do you want to count?
### Task 7.1:
Write a loop that prints every third number starting at 15 and going down to 6 (so 6 is included in the loop).

Woot, woot! Making progress. :) Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
You can also nest loops inside one another. Let's say we want to do a simple summation program. Imagine we want to sum each number from 1 to 4 with each number from 1 to 4. So, we essentially want to do this:

```
num1	+	num2	=	Sum
1	+	1	=	2
1	+	2	=	3
1	+	3	=	4
1	+	4	=	5
2	+	1	=	3
2	+	2	=	4
2	+	3	=	5
2	+	4	=	6
```

At the end, we will have 16 sums (4x4).

In code, we can find these sums using two for loops, one nested inside the other. Let's think of num1 as the value of one loop and num2 as the value of the other loop. We know from the examples above how to generate a loop that generates the numbers from 1 to 4, so we will do this for num1:

```
for (int num1=1; num1<5; num1++){
   //put code here
}
```

We know that for each value of num1, we want to sum it with all values of num2, which are the numbers 1 to 4. So we can use another for loop to generate those numbers. Since we want to generate all the values of num2 for each value of num1, let's put the num2 for loop inside of the num1 for loop, like this:

```
for (int num1=1; num1<5; num1++){  // num1 for-loop
   for (int num2=1; num2<5; num2++){  //num2 for-loop
      cout << num1 << " + " << num2 << " = " << num1+num2 << "\n";  //prints out num1+num2
   }
}
```

And this gives us the output:

```
1 + 1 = 2
1 + 2 = 3
1 + 3 = 4
1 + 4 = 5
2 + 1 = 3
2 + 2 = 4
2 + 3 = 5
2 + 4 = 6
3 + 1 = 4
3 + 2 = 5
3 + 3 = 6
3 + 4 = 7
4 + 1 = 5
4 + 2 = 6
4 + 3 = 7
4 + 4 = 8
```

Notice that the first number that prints out is the num1 variable, and the second number is the num2 variable, and the final number is the sum of num1 and num2. Let's quickly talk through the loop. We begin with num1 being equal to 1 and num2 being equal to 1, and num1+num2 equaling 2. Then the num2 loop advances by 1, because the inner loop has to finish before the outer loop can advance. So num1 is 1 and num2 is 2 and their sum is 3. Then num2 advances by 1 again, and num1 is 1 and num2 is 3 and their sum is 3. Finally, num1 is 1 and num2 is 4 and their sum is 5. The next time num2 is incremented it becomes 5, violating the less than 5 condition of the loop, and so the num2 loop finishes. Now the num1 loop advances by 1, so num1 is 2. Now the num2 loop starts all over again from the beginning, and num2 is 1 and num1+num2 is 3, and so forth and so on. Nested loops can be a little tricky, so if they are confusing, ask for help!

### Task 7.2:
Write a program that multiplies all the odd numbers between 1 and 9 by all the even numbers between 2 and 10. Hint: follow the example above, but don't forget to change your loop initializations.

Good work! Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
We typically use while loops when we don't know how many times the loop should execute. For example, let's say you are trying to get input from a user that is a number between 1 and 10. while the user keeps typing in numbers that are too large or too small, you can ask her for new input until she enters a number between 1 and 10. Here's what this while loop would look like:

```
int main(){
   int userInput;

   //get input from user
   cout << "Enter a number between 1 and 10: ";
   cin >> userInput;

   //as long as userInput is less than 1 or greater than 10
   while((userInput < 1) || (userInput > 10)){
      cout << "Read directions!  Enter a number between 1 and 10: ";
      cin >> userInput;  //get new input from the user
   }
   //when we reach this line, we know the user entered a number between 1 and 10,
   //or else we would still be stuck in the while loop.
   cout << "You entered " << userInput << ".\n";
}
```

You'll notice that the logical expressions appear again—this time in the parentheses after the while. Once again, the loop will continue as long as this expression is true. Let's walk through what is happening in this program. We first get some input from the user. We begin the while loop by checking the input to see if it is less than 1 or greater than 10. If it is, we yell at the user and ask for new input, and we make sure to save the new input to userInput. (Why is this so important? We'll see in a bit.) Then the program loops back to the beginning of the while loop and checks to see if userInput is less than 1 or greater than 10. If it still is, we execute the loop again and ask for new input. We continue this as many times as necessary until the user cooperates. As soon as the user enters a number between 1 and 10, we break out of the while loop, because we only execute the while loop if userInput is less than 1 or greater than 10. We then print out the number.

In the above program, we made sure to save the new user input to the same variable as the old user input. This step is important because that's the variable being evaluated in the logical expression—if we didn't update it, the while loop would never stop! To illustrate this, let's change the above while loop to make it incorrect by introducing another variable called newUserInput:

```
while((userInput < 1) || (userInput > 10)){
   cout << "Read directions!  Enter a number between 1 and 10: ";
   cin >> newUserInput;  //get new input from the user and save it to new variable
}
```

This loop begins by checking to see if userInput is less than 1 or greater than 10. Let's say that userInput equals 100, so the loop will run. Inside the loop, the program prints out a nasty message to the user and gets new input from the user, the number 5. This time, however, 5 is saved to a different variable called newUserInput. Then the program checks the loop condition again. Since we never changed the value in userInput, it still equals 100, so the loop will run again even though the user just entered a valid number. In fact, this loop will run forever! This is called an infinite loop. To avoid this, always make sure that you update the loop variable somehow by changing its value so that eventually your loop will stop. (To break out of an infinite loop, type Ctrl-C.)

### Task 7.3:
Create a program that asks the user to enter a number between 5 and 50. If the user enters invalid input, print out a message to the user asking them to try again and keep getting input from the user until they enter a valid number. Bonus: Can you keep track of the number of attempts a user makes to enter a number, and then print it out when they finally enter a valid number?

Good job! You are almost finished. For your last lesson, you'll learn how to use arrays.
But first, don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!

## Section 8: Arrays
Sometimes we want to store lists of data together in one place. For example, it might be handy to have a list of all your grades from science class. You could go back later and average all the grades in the list together to find out your final grade in science. In C++, lists are called arrays, and they have some special properties.

You can think of an array like a motel. Let's begin by imagining a single-story motel complex. The rooms are laid out in a row right next to each other. If you stand in front of room 7, you know that room 8 will be right next door, and room 9 will be after room 8. Arrays work in the same way. We can put values inside each of the "rooms" in the array, and since they are numbered logically, we can quickly find any "room" we need.

When you build a motel, you have to decide ahead of time how many rooms you want there to be, and you can't change your mind after the motel is built (without paying a lot of construction costs). In a C++ array, at the beginning of the function you declare the type, name, and size of your array. Going back to the science grade example, we would declare a science grade array like this:

`float gradeArray[5];`

The brackets with the number mean that we are constructing an array that is five units in length named gradeArray. We declare it as a float because we plan on storing floating point numbers in each of the "rooms" in the array. We can substitute the number 5 with any length of array we want, but once we declare our array to be a certain size, we can't change it.
Now that we've declared an array, we need to put things inside it. Inside our main function, we'll enter five grades into the array. When you put a value into an array, you have to tell the computer where to put it, just like the front desk in a motel would assign you a room. Like rooms in a motel, locations, or elements, in an array are numbered—in an array, though, the numbers are from 0 to one less than the size of the array. In gradeArray, then, there are five elements, which are numbered 0, 1, 2, 3, and 4. We talk about a specific elements of an array by writing the name of the array and then putting the location number in brackets, like this: gradeArray[0] . Let's put some values into them in the following code:

```
gradeArray[0] = 98.3;
gradeArray[1] = 91.1;
gradeArray[2] = 89.8;
gradeArray[3] = 90.5;
gradeArray[4] = 95.0;
```
Now that we have our grades in an array, we want to calculate our final grade by averaging them all together. To do so, we need to visit each element in the array (imagine going from door to door in a motel) and add together the values in each position. Finally, we need to divide them by the number of values in the array. To do this systematically, we can use a for loop. We will loop through each element in the array and add its value to a variable called gradeSum that we initialize ahead of time. After we finish the loop, we will divide gradeSum by 5 to get the average. Here's the code for this program:

```
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

### Task 8.1:
Change the program above so that it stores and uses 7 grades instead of 5 to calculate the average.

Yay! Two more :) Don't forget to turn your cups to red so that a member of the camp staff can check your code, and don't forget to switch drivers!
Task 8.2:
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

### Task 8.3:
Determine the correct code to access the elements containing letters y and z in the diagram above.

Congratulations! You have finished the C++ tutorial!
Don't forget to turn your cups to red so that a member of the camp staff can check your code.
Once your project team is finished, one of the camp staff will bring you the materials for your project and explain how to get started. Until then, you can explore coding and the Linux environment, and you can let your other team members know that you are available if they have questions.
