#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <limits>
#include <algorithm>
#include <chrono>
#include <future>
#include <thread>
#include <boost/python.hpp>

short DEPTH = 20;

std::vector<std::string> get_legal_moves(std::array<int, 6> state, bool maximizing) {
    std::vector<std::string> moves;

    if (maximizing) {
        // reload case
        if (state[1] != 10) {
            moves.push_back("RELOAD");
        }
        // throw case
        if (state[1] != 0) {
            moves.push_back("THROW");
        }
        // duck case
        if (state[2] != 0) {
            moves.push_back("DUCK");
        }
    } else {
        // reload case
        if (state[4] != 10) {
            moves.push_back("RELOAD");
        }
        // throw case
        if (state[4] != 0) {
            moves.push_back("THROW");
        }
        // duck case
        if (state[5] != 0) {
            moves.push_back("DUCK");
        }
    }
    return moves;
}

std::array<int, 6> apply_move(std::array<int, 6> state, std::string move, std::string oppMove, bool maximizing) {
    int i;
    std::array<int, 6> tstate;
    for (i = 0; i < 6; i++) {
        tstate[i] = state[i];
    }

    if (maximizing) {
        if (move == "RELOAD") {
            tstate[1] += 1;
        } else if (move == "THROW") {
            tstate[1] -= 1;
        } else {
            tstate[2] -= 1;
        }
    } else {
        if (move == "RELOAD") {
            tstate[4] += 1;
            if (oppMove == "THROW") {
                tstate[0] += 1;
            }
        } else if (move == "THROW") {
            tstate[4] -= 1;
            if (oppMove == "RELOAD") {
                tstate[3] += 1;
            }
        } else {
            tstate[5] -= 1;
        }
    }
    return tstate;
}

bool is_terminal(std::array<int, 6> state, int rounds, short depth) {
    if (state[0] == 3 || state[3] == 3 || rounds + DEPTH - depth == 30) {
        return true;
    }
    return false;
}

float pointCurve[3] = {1, 4, 9};
float snowballCurve[11] = {-3, 0, 1, 2, 3, 4, 5, 5.5, 6, 6.5, 7};
float duckCurve[6] = {-5, 0, 0.5, 1, 2, 3};

float eval(std::array<int, 6> state, short depth, int rounds) {
    if (rounds + DEPTH - depth == 30) {
        if (state[0] > state[3]) {
            return std::numeric_limits<float>::max();
        } else {
            return -std::numeric_limits<float>::max();
        }
    } else if (state[0] == 3) {
        return std::numeric_limits<float>::max();
    } else if (state[3] == 3) {
        return -std::numeric_limits<float>::max();
    }

    return pointCurve[state[0]] + snowballCurve[state[1]] + duckCurve[state[2]] - pointCurve[state[3]] -
           snowballCurve[state[4]] - duckCurve[state[5]];
}

float
minimax(std::array<int, 6> state, float a, float b, bool maximizing, std::string oppMove, short depth, int rounds) {
    float value;
    int i;
    std::vector<std::string> moves;
    std::array<int, 6> tstate;

    if (depth == 0 || is_terminal(state, rounds, depth)) {
        return eval(state, depth, rounds);
    } else {
        if (maximizing) {
            value = -std::numeric_limits<float>::max();
            moves = get_legal_moves(state, true);
            for (i = 0; i < moves.size(); i++) {
                tstate = apply_move(state, moves[i], "", true);
                value = std::max(value, minimax(tstate, a, b, false, moves[i], depth - 1, rounds));
                a = std::max(a, value);
                if (a > b) {
                    return value;
                }
            }
            return value;
        } else {
            value = std::numeric_limits<float>::max();
            moves = get_legal_moves(state, false);
            for (i = 0; i < moves.size(); i++) {
                tstate = apply_move(state, moves[i], oppMove, false);
                value = std::min(value, minimax(tstate, a, b, true, "", depth - 1, rounds));
                b = std::min(b, value);
                if (a > b) {
                    return value;
                }
            }
            return value;
        }
    }
}

// MP
int mpminimax(std::array<int, 6> state, float a, float b, bool maximizing, std::string oppMove, short depth, int rounds,
              std::promise<float> &&p) {
    float value;
    int i;
    std::vector<std::string> moves;
    std::array<int, 6> tstate;

    if (depth == 0 || is_terminal(state, rounds, depth)) {
        p.set_value(eval(state, depth, rounds));
        return 0;
    } else {
        if (maximizing) {
            value = -std::numeric_limits<float>::max();
            moves = get_legal_moves(state, true);
            for (i = 0; i < moves.size(); i++) {
                tstate = apply_move(state, moves[i], "", true);
                value = std::max(value, minimax(tstate, a, b, false, moves[i], depth - 1, rounds));
                a = std::max(a, value);
                if (a > b) {
                    p.set_value(value);
                    return 0;
                }
            }
            p.set_value(value);
            return 0;
        } else {
            value = std::numeric_limits<float>::max();
            moves = get_legal_moves(state, false);
            for (i = 0; i < moves.size(); i++) {
                tstate = apply_move(state, moves[i], oppMove, false);
                value = std::min(value, minimax(tstate, a, b, true, "", depth - 1, rounds));
                b = std::min(b, value);
                if (a > b) {
                    p.set_value(value);
                    return 0;
                }
            }
            p.set_value(value);
            return 0;
        }
    }
}
// END MP

std::string search(std::array<int, 6> state, int rounds) {
    int i;
    float a, b, sVal;
    a = -std::numeric_limits<float>::max();
    b = std::numeric_limits<float>::max();
    std::string bestMove;
    std::vector<std::string> moves = get_legal_moves(state, true);
    std::array<int, 6> tstate;
    std::array<std::future<float>, 3> futures;
    std::array<std::thread, 3> threads;

    float value = -std::numeric_limits<float>::max();

    for (i = 0; i < moves.size(); i++) {
        std::promise<float> p;
        futures[i] = p.get_future();
        tstate = apply_move(state, moves[i], "", true);
        threads[i] = std::thread(&mpminimax, std::move(tstate), a, b, std::move(false), moves[i], DEPTH, rounds,
                                 std::move(p));
        //mpminimax(tstate, a, b, false, moves[i], DEPTH, rounds);
    }

    for (i = 0; i < moves.size(); i++) {
        threads[i].join();
        sVal = futures[i].get();
        //a = std::max(a, value);
        if (sVal > value) {
            bestMove = moves[i];
            value = sVal;
        }
    }
    return bestMove;
}

std::string getMove(boost::python::object imyScore, boost::python::object imySnowballs, boost::python::object imyDucks,
                    boost::python::list lmyMoves,
                    boost::python::object ieScore, boost::python::object ieSnowballs, boost::python::object ieDucks,
                    boost::python::list leMoves) {
    int myScore, mySnowballs, myDucks, eScore, eSnowballs, eDucks;
    myScore = boost::python::extract<int>(imyScore);
    mySnowballs = boost::python::extract<int>(imySnowballs);
    myDucks = boost::python::extract<int>(imyDucks);
    eScore = boost::python::extract<int>(ieScore);
    eSnowballs = boost::python::extract<int>(ieSnowballs);
    eDucks = boost::python::extract<int>(ieDucks);

    //boost::python::list lmyMoves, leMoves;
    //lmyMoves = static_cast<boost::python::list>(boost::python::extract<boost::python::list>(imyMoves));
    //leMoves = static_cast<boost::python::list>(boost::python::extract<boost::python::list>(ieMoves));

    std::vector<std::string> myMoves;
    std::vector<std::string> eMoves;
    lmyMoves.reverse();
    leMoves.reverse();
    int len;
    len = boost::python::len(lmyMoves);
    for (int i = 0; i < len; i++) {
        myMoves.push_back(boost::python::extract<std::string>(lmyMoves.pop()));
    }
    len = boost::python::len(leMoves);
    for (int i = 0; i < len; i++) {
        eMoves.push_back(boost::python::extract<std::string>(leMoves.pop()));
    }
    std::string bestMove;
    int rounds = myMoves.size();
    myDucks = 5 - myDucks;
    eDucks = 5 - eDucks;
    std::array<int, 6> state = {myScore, mySnowballs, myDucks, eScore, eSnowballs, eDucks};

    bestMove = search(state, rounds);

    return bestMove;
}

BOOST_PYTHON_FUNCTION_OVERLOADS(getMove_overloads, getMove, 8, 8);

BOOST_PYTHON_MODULE (stocksnow) {
    boost::python::def("getMove", &getMove, getMove_overloads(
            (boost::python::arg("myScore"),
                    boost::python::arg("mySnowballs"),
                    boost::python::arg("myDucks"),
                    boost::python::arg("myMoves"),
                    boost::python::arg("eScore"),
                    boost::python::arg("eSnowballs"),
                    boost::python::arg("eDucks"),
                    boost::python::arg("eMoves"))));
}
/**
int main() {
    std::vector<std::string> history = {};
    std::chrono::high_resolution_clock::time_point begin, end;
    std::string move;
    begin = std::chrono::high_resolution_clock::now();
    move = getMove(0, 1, 0, history, 0, 1, 0, history);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
    std::cout << "Found move: " << move << " in " << time_span.count() << " seconds." << std::endl;
    return 0;
}**/
