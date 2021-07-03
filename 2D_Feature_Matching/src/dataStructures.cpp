#include "dataStructures.h"
#include <iostream>

using namespace std;

void deleteNode(Node **head)
{
    if(*head ==NULL)
    {
        return;
    }
    Node *temp = *head;
    *head = temp->next;
    delete temp;
    return;
}
void insert(Node **head, DataFrame &data, int bufferSize)
{
    struct Node* temp = new Node;
    temp->dataFrame = data;
    temp-> next = NULL;
    struct Node *last = *head;
    if(*head==NULL)
    {
        *head = temp;
        temp->num = 0;
        return;
    }
    int num=1;
    while(last->next !=NULL)
    {
        num++;
        last = last -> next;

        if(num==bufferSize)
        {
            deleteNode(head);
            num--;
        }

    }
    temp->num = (last->num) +1;
    last->next = temp;
    return;
}

void display(Node **head)
{
    
    if(*head==NULL)
    {
        cout<<"List is empty"<<endl;
        return;
    }
    
    Node *temp = *head;
    while(temp != NULL)
    {

        cout<<temp->num<<endl;
        temp = temp->next;
        
    }
    
    return;
}

