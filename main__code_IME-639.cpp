#include <ilcplex/ilocplex.h>
#include<iostream>
#include<fstream>
#include<cstdlib>
#include<utility>
#include<algorithm>
#include<math.h>
using namespace std;
ILOSTLBEGIN
/* Helper function for adding constraints and objective function to the lp*/
static void populatebyrow(IloModel model, IloNumVarArray var, IloRangeArray con);
/* Declaring all the variables for the problem */
int number_of_nodes;
int number_of_vehicles;
int number_of_lines;
int number_of_windows;
int headway[100][2 * 100];
int break_capacity[100];
int number_of_feasible_trips;
int st[100];
int et[100];
int sn[100];
int en[100];
int sl[100];
int el[100];
int min_stopping_time[100];
int max_stopping_time[100];
int pull_out_trip[100];
int pull_in_trip[100];
int break_cost[100];


int main(void)
{
    IloEnv env;
    try
    {
        IloModel model(env);

        IloNumVarArray var(env);
        IloNumVarArray time_var(env);
        //time_var is the travel time of a deadhead trip from a end of trip node i 
        //to the start of trip j
        IloRangeArray con(env);
        populatebyrow(model, var, con);
        /*scanning  all the inputs */
        ifstream in("D:\\cplex_project\\project2\\input.txt");
        {
            if (in)
            {
                in >> number_of_nodes;
                in >> number_of_vehicles;
                in >> number_of_lines;
                in >> number_of_windows;
                for (int i = 0; i < number_of_windows; i++)
                {
                    for (int j = 0; j < 2 * number_of_lines; j++)
                    {
                        in >> headway[i][j];
                    }
                }
                for (int i = 0; i < number_of_nodes + 1; i++)
                    in >> break_capacity[i];

                in >> number_of_feasible_trips;

                for (int i = 0; i < number_of_feasible_trips; i++)
                    in >> st[i];

                for (int i = 0; i < number_of_feasible_trips; i++)
                    in >> et[i];

                for (int i = 0; i < number_of_feasible_trips; i++)
                    in >> sn[i];

                for (int i = 0; i < number_of_feasible_trips; i++)
                    in >> en[i];

                for (int i = 0; i < number_of_lines; i++)
                    in >> sl[i];

                for (int i = 0; i < number_of_lines; i++)
                    in >> el[i];

                for (int i = 0; i < number_of_windows; i++)
                    in >> min_stopping_time[i];

                for (int i = 0; i < number_of_windows; i++)
                    in >> max_stopping_time[i];

                for (int i = 0; i < number_of_nodes; i++)
                    in >> pull_out_trip[i];

                for (int i = 0; i < number_of_nodes; i++)
                    in >> pull_in_trip[i];

                for (int i = 0; i < number_of_nodes; i++)
                    in >> break_cost[i];
            }
            else
            {
                cerr << "No such file: " << "D:\\cplex_project\\project2\\input.txt" << endl;
                throw(1);
            }
        }


        IloCplex cplex(model);
        cplex.solve();

        env.out() << "Solution status = " << cplex.getStatus() << endl;
        env.out() << "Solution value  = " << cplex.getObjValue() << endl;

        IloNumArray vals(env);
        cplex.getValues(vals, var);
        env.out() << "Values        = " << vals << endl;
        // cplex.getSlacks(vals, con);
        // env.out() << "Slacks        = " << vals << endl;
        /* Printing output for the program*/
        for (int i = 0; i < number_of_feasible_trips; i++)
        {
            for (int j = 0; j < number_of_feasible_trips; j++)
            {
                for (int k = 0; k < number_of_vehicles; k++)
                {
                    if (vals[i * number_of_feasible_trips + j * number_of_feasible_trips + k] == true)
                        cout << "Vehicle " << k << " will be doing the trip " << j << " after completing the trip " << i << endl;
                }
            }
        }
        /* printing which vehicle is covering which trip */
        for (int i = 0; i < number_of_feasible_trips; i++)
        {
            for (int k = 0; k < number_of_vehicles; k++)
            {
                if (vals[i * number_of_feasible_trips + k] == true)
                    cout << "Vehicle " << k << " will do the trip " << i << endl;
            }
        }
    }
    catch (IloException& e)
    {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) 
    {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();
    return 0;

}


static void
populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c)
{
    /*adding yik to the problem */
    IloEnv env = model.getEnv();
    for (int i = 0; i < number_of_feasible_trips; i++)
        for (int j = 0; j < number_of_vehicles; j++)
            x.add(IloNumVar(env, 0.0, 1.0, ILOINT));

    /*adding xijk decision variables to the problem */
    for (int i = 0; i < number_of_feasible_trips; i++)
    {
        for (int j = 0; j < number_of_feasible_trips; j++)
        {
            if (j != i)
            {
                for (int k = 0; k < number_of_vehicles; k++)
                    x.add(IloNumVar(env, 0.0, 1.0, ILOINT));
            }
        }
    }
    //adding the summation yik <=1 constraint
    for (int i = 0; i < number_of_feasible_trips; i++)
    {
        IloExpr row(env);
        for (int j = i * number_of_vehicles; j < (i + 1) * number_of_vehicles; j++)
            row += x[j];
        c.add(row <= 1);
    }

    /*xijk=yik and xijk = yjk */
 
    for (int i = 0; i < number_of_feasible_trips; i++)
    {
        for (int j = 0; j < number_of_feasible_trips; j++)
        {
            if (j != i)
            {
                for (int k = 0; k < number_of_vehicles; k++)
                {
                    c.add(x[number_of_feasible_trips * number_of_vehicles + i * number_of_feasible_trips + j * number_of_feasible_trips + k] - x[i * k] == 0);
                    c.add(x[number_of_feasible_trips * number_of_vehicles + i * number_of_feasible_trips + j * number_of_feasible_trips + k] - x[j * k] == 0);
                }
            }
        }
    }
    /* adding headway constraint to the problem */
    for (int i = 0; i < number_of_feasible_trips; i++)
    {
        int minVal = INT_MAX;
        int conse_trip = 0;
        for (int j = 0; j < number_of_feasible_trips; j++)
        {
            if (sn[i] == sn[j] && en[i] == en[j] && j != i)
            {
                if (abs(st[j] - et[i]) <= minVal)
                {
                    conse_trip = j;
                    minVal = abs(st[j] - et[i]);
                }
            }
        }
        // assuming the time to be 24 hrs and equally divided into number of time windows 
        int length_of_each_duration = 24 / number_of_windows;
        int i_timewindow;
        for (int ii = 0; ii < number_of_windows; ii++)
        {
            if ((ii * length_of_each_duration < et[i]) && ((ii + 1) * length_of_each_duration >= et[i]))
                i_timewindow = ii;
        }
        int j_timewindow;
        for (int ii = 0; ii < number_of_windows; ii++)
        {
            if ((ii * length_of_each_duration < st[conse_trip]) && ((ii + 1) * length_of_each_duration >= st[conse_trip]))
                j_timewindow = ii;
        }
        int line;
        for (int ik = 0; ik < number_of_lines; ik++)
        {
            if ((sl[ik] == sn[i]) && (el[ik] == en[i]))
                line = ik;
        }
        /*adding min stopping time constraint and max stopping time constraint to the model
        */
        IloNumExpr rows(env);
        for (int q = 0; q < number_of_vehicles; q++)
            rows += x[number_of_feasible_trips * number_of_vehicles + i * number_of_feasible_trips + conse_trip * number_of_feasible_trips + q];
        if (j_timewindow == i_timewindow)
        {
            if (sn[i] < en[i])
                c.add(rows * abs(st[conse_trip] - et[i]) - rows * headway[i_timewindow][line + 1] <= 0);
            else
                c.add(rows * abs(st[conse_trip] - et[i]) - rows * headway[i_timewindow][line] <= 0);

        }
        else
        {
            if (sn[i] < en[i])
                c.add(rows * abs(st[conse_trip] - et[i]) - rows * max(headway[i_timewindow][line + 1], headway[j_timewindow][line + 1]) <= 0);
            else
                c.add(rows * abs(st[conse_trip] - et[i]) - rows * max(headway[i_timewindow][line], headway[j_timewindow][line]) <= 0);
        }

        if (en[i] == sn[conse_trip])
        {
            c.add(rows * abs(st[conse_trip] - et[i])  - rows * max(max_stopping_time[i_timewindow], max_stopping_time[j_timewindow]) <= 0);
            c.add(rows * abs(st[conse_trip] - et[i])  - rows * min(min_stopping_time[i_timewindow], min_stopping_time[j_timewindow]) >= 0);
        }
        else
        {
            c.add(rows * pull_out_trip[sn[conse_trip]] + rows * pull_in_trip[en[i]] + rows * max(max_stopping_time[i_timewindow], max_stopping_time[j_timewindow]) - rows * abs(st[conse_trip] - et[i]) >= 0);
        }

    }


    IloNumExpr objc(env);
    /* Adding the objective function to our problem summation of the total cost */
    for (int i = 0; i < number_of_feasible_trips; i++)
    {
        for (int j = 0; j < number_of_feasible_trips; j++)
        {
            for (int k = 0; k < number_of_vehicles; k++)
                objc += abs(st[j] - et[i]) * x[number_of_feasible_trips * number_of_vehicles + i * number_of_feasible_trips + j * number_of_feasible_trips + k];
        }
    }
    for (int i = 0; i < number_of_feasible_trips; i++)
    {
        for (int k = 0; k < number_of_vehicles; k++)
            objc += x[i * number_of_vehicles + k] * break_cost[en[i]];
    }
    model.add(IloMinimize(env, objc));

} // END populatebyrow
