#include <gtkmm.h>
#include <iostream>

using namespace std;

class myLabel: public Gtk::Window
{
public:
    myLabel();
    virtual ~myLabel();

protected:
    Gtk::Label m_label;
    string labeltext;
    string newtext;
    void myprocess1();
};

myLabel::myLabel() :
        m_label()
{
    void myprocess1();

    set_title("Gtkmm Programming - C++");
    add(m_label);

    m_label.show();

    Glib::Thread::create(sigc::mem_fun(*this, &myLabel::myprocess1), true);
}

myLabel::~myLabel()
{
}

void myLabel::myprocess1()
{
    labeltext = "About to preform a number of processes.\n";
    labeltext += "Each process may take up to three hours.\n";
    labeltext += "Please carry your daily chores and wait.\n";
    cout << labeltext;
    cout.flush();
    m_label.set_text(labeltext);

    sleep(10); // Back from a three hour function
    newtext = "Back from a three hour function\n";
    labeltext += newtext;
    m_label.set_text(labeltext);
    cout << newtext;
    cout.flush();

    sleep(10); // Back from a three hour function
    newtext = "Back from another three hour function\n";
    labeltext += newtext;
    m_label.set_text(labeltext);
    cout << newtext;
    cout.flush();

    newtext = "Exiting in 1 minute...\n";
    labeltext += newtext;
    m_label.set_text(labeltext);
    cout << newtext;
    cout.flush();
    sleep(60);
    exit(0);
}

int main(int argc, char* argv[])
{
    if (Glib::thread_supported())
        Glib::thread_init();
    else
    {
        cerr << "Threads aren't supported!" << endl;
        exit(1);
    }

    Gtk::Main kit(argc, argv);

    myLabel mylabel;
    Gtk::Main::run(mylabel);
    return 0;
}