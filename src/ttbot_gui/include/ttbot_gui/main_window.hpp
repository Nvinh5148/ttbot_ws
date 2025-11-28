#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <vector>
#include <utility>
#include "ros_worker.hpp"

// Namespace này được tạo tự động từ file .ui
namespace Ui { class MainWindow; }

// Widget Map tùy chỉnh để bắt click chuột
class MapView : public QGraphicsView {
    Q_OBJECT
public:
    explicit MapView(QWidget* parent = nullptr);
    void setImage(const QString& path);
    void clearVisuals();
signals:
    void mapClicked(double x, double y);
protected:
    void mousePressEvent(QMouseEvent* event) override;
private:
    QGraphicsScene* scene_;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(RosWorker* worker, QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onMapClicked(double px, double py);
    void onBtnPublishClicked(); // Slot cho nút Publish
    void onBtnClearClicked();   // Slot cho nút Clear

private:
    Ui::MainWindow *ui;         // Con trỏ quản lý giao diện kéo thả
    RosWorker* ros_worker_;
    MapView* custom_map_view_;

    std::vector<std::pair<double, double>> path_points_;
    std::pair<double, double> last_click_px_;

    // CẤU HÌNH MAP (CẦN CHỈNH LẠI SAU)
    double MAP_RESOLUTION = 0.05; 
    double ORIGIN_X_PX = 400.0;
    double ORIGIN_Y_PX = 300.0;
};
#endif