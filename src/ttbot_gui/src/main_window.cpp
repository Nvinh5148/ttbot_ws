#include "ttbot_gui/main_window.hpp"
#include "ui_mainwindow.h" // FILE NÀY TỰ SINH RA, KHÔNG CẦN TẠO
#include <QGraphicsPixmapItem>
#include <QDebug>

// === MAP VIEW LOGIC ===
MapView::MapView(QWidget* parent) : QGraphicsView(parent) {
    scene_ = new QGraphicsScene(this);
    this->setScene(scene_);
}
void MapView::setImage(const QString& path) {
    QPixmap pixmap(path);
    if (!pixmap.isNull()) {
        scene_->clear();
        scene_->addPixmap(pixmap);
        this->setSceneRect(pixmap.rect());
    } else {
        qDebug() << "ERROR: Cannot load map image at" << path;
    }
}
void MapView::clearVisuals() {
    QList<QGraphicsItem*> items = scene_->items();
    for (auto item : items) {
        if (item->type() != QGraphicsPixmapItem::Type) {
            scene_->removeItem(item);
            delete item;
        }
    }
}
void MapView::mousePressEvent(QMouseEvent* event) {
    QPointF pt = mapToScene(event->pos());
    emit mapClicked(pt.x(), pt.y());
    scene_->addEllipse(pt.x()-3, pt.y()-3, 6, 6, QPen(Qt::red), QBrush(Qt::red));
    QGraphicsView::mousePressEvent(event);
}

// === MAIN WINDOW LOGIC ===
MainWindow::MainWindow(RosWorker* worker, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), ros_worker_(worker)
{
    ui->setupUi(this); // Load giao diện từ file .ui bạn vẽ

    // Hack: Thay thế khung map_view trống trong Designer bằng MapView xịn của code
    custom_map_view_ = new MapView(this);
    custom_map_view_->setGeometry(ui->map_view->geometry());
    custom_map_view_->setParent(ui->centralwidget);
    ui->map_view->setVisible(false); // Ẩn cái cũ đi

    // Load Map (SỬA ĐƯỜNG DẪN ẢNH CỦA BẠN Ở ĐÂY)
    QString mapPath = "/home/uylegia/ttbot_ws/src/ttbot_gui/resources/map.png";
    custom_map_view_->setImage(mapPath);

    // Kết nối nút bấm từ UI với hàm xử lý
    // ui->btn_publish: tên bạn đặt trong Qt Creator
    connect(custom_map_view_, &MapView::mapClicked, this, &MainWindow::onMapClicked);
    connect(ui->btn_publish, &QPushButton::clicked, this, &MainWindow::onBtnPublishClicked);
    connect(ui->btn_clear, &QPushButton::clicked, this, &MainWindow::onBtnClearClicked);

    last_click_px_ = {-1.0, -1.0};
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::onMapClicked(double px, double py) {
    double dx = (ORIGIN_Y_PX - py) * MAP_RESOLUTION;
    double dy = (ORIGIN_X_PX - px) * MAP_RESOLUTION;
    path_points_.push_back({dx, dy});
    
    // Vẽ đường đỏ nối các điểm
    if (last_click_px_.first >= 0) {
        QPen pen(Qt::red); pen.setWidth(3);
        custom_map_view_->scene()->addLine(last_click_px_.first, last_click_px_.second, px, py, pen);
    }
    last_click_px_ = {px, py};
}

void MainWindow::onBtnPublishClicked() {
    if (!path_points_.empty()) ros_worker_->publishPath(path_points_);
}

void MainWindow::onBtnClearClicked() {
    path_points_.clear();
    last_click_px_ = {-1.0, -1.0};
    custom_map_view_->clearVisuals();
}