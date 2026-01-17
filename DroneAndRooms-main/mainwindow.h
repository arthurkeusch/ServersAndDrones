#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void update();

    void on_actionShow_graph_triggered(bool checked);

    void on_actionMove_drones_triggered();

    void on_actionQuit_triggered();

    void on_actionCredits_triggered();

    void on_actionLoad_triggered();

private:
    /**
     * @brief loadJson
     * @param title
     * @return
     */
    bool loadJson(const QString& title);
    void createVoronoiMap();

    /**
     * @brief Builds the server adjacency graph.
     *
     * For each pair of servers, a link is created if their Voronoï areas
     * share a common edge. The link length corresponds to the distance
     * between the two servers through the middle of this shared edge.
     */
    void createServersLinks();

    /**
     * @brief Computes all-pairs shortest paths between servers.
     *
     * Uses Dijkstra's algorithm from each server to compute:
     *  - the minimal distance to every other server
     *  - the first link to take to follow the shortest path
     * Results are stored in server.bestDistance.
     */
    void fillDistanceArray();

    Ui::MainWindow *ui;
    QVector<QVector<float>> distanceArray;

    // to animate drones
    QTimer *timer;
    QElapsedTimer elapsedTimer;
};
#endif // MAINWINDOW_H
