#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QTableWidget>
#include <QHeaderView>
#include <QComboBox>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTimer>
#include <QElapsedTimer>
#include <QUuid>
#include <QScrollArea>
#include <QGroupBox>
#include <QMainWindow>
#include <QDockWidget>
#include <map>

namespace agv_rviz {
class MissionPanel : public rviz_common::Panel {
  Q_OBJECT
  friend class MissionPanelTest;
public:
  explicit MissionPanel(QWidget *parent=nullptr):Panel(parent) {
    setObjectName("agvMissionPanel");setMinimumWidth(320);
    auto outer=new QVBoxLayout(this);outer->setContentsMargins(4,4,4,4);
    auto scroll=new QScrollArea;scroll->setWidgetResizable(true);outer->addWidget(scroll);
    auto contents=new QWidget;scroll->setWidget(contents);auto layout=new QVBoxLayout(contents);
    connection_=new QLabel("等待巡检后端连接…");connection_->setObjectName("connectionStatus");layout->addWidget(connection_);
    auto box=new QGroupBox("采集区域 · 长度沿道路方向");auto form=new QFormLayout(box);layout->addWidget(box);
    id_=new QLineEdit("inspection");id_->setObjectName("mission_id");form->addRow("任务名称",id_);
    connect(id_,&QLineEdit::textEdited,this,[this]{dirty_=true;refresh();});
    addField(form,"start_x","起点 X / m",6,-100000,100000,3);
    addField(form,"start_y","起点 Y / m",-1,-100000,100000,3);
    addField(form,"length","采集长度 / m",3,.01,10000,3);
    addField(form,"width","采集宽度 / m",2,.01,1000,3);
    addField(form,"spacing","轨道间距 / m",1,.1,1.5,3);
    addField(form,"speed","采集速度 / km/h",1.8,.18,10.,2);
    addField(form,"error","覆盖误差预算 / m",.1,0,.7,3);
    auto note=new QLabel("幅宽 1.5 m · 默认 4096 × 4096 行\n原图为灰度；平场与畸变校正离线进行");note->setWordWrap(true);layout->addWidget(note);
    auto row=new QGridLayout;layout->addLayout(row);
    button(row,0,0,"load","载入请求",[this]{fileAction("load","载入区域请求","YAML (*.yaml *.yml)");});
    button(row,0,1,"save","保存请求",[this]{auto p=QFileDialog::getSaveFileName(this,"保存新请求",QString(),"YAML (*.yaml)");if(!p.isEmpty())send("save",{{"path",p},{"fields",fields()}});});
    button(row,1,0,"preview","预览轨迹",[this]{send("preview",{{"fields",fields()}});});
    button(row,1,1,"prepare","准备任务",[this]{send("prepare",{{"fields",fields()}});});
    button(row,2,0,"start","开始采集",[this]{send("start");});
    button(row,2,1,"pause","暂停",[this]{send("pause");});
    button(row,3,0,"resume","继续采集",[this]{send("resume");});
    button(row,3,1,"cancel","取消并停车",[this]{send("cancel");});
    buttons_["cancel"]->setStyleSheet("QPushButton:enabled { color: #c53b37; font-weight: bold; }");
    state_=new QLabel("任务：未准备");state_->setObjectName("missionState");state_->setWordWrap(true);layout->addWidget(state_);
    progress_=new QProgressBar;progress_->setFormat("任务步骤 %v / %m");layout->addWidget(progress_);
    message_=new QLabel;message_->setWordWrap(true);message_->setTextInteractionFlags(Qt::TextSelectableByMouse);message_->setObjectName("operationMessage");layout->addWidget(message_);
    auto coverageBox=new QGroupBox("覆盖检查与补扫");auto cl=new QVBoxLayout(coverageBox);layout->addWidget(coverageBox);
    auto cr=new QGridLayout;cl->addLayout(cr);
    button(cr,0,0,"audit","审计本次采集",[this]{send("audit");});
    button(cr,0,1,"load_coverage","载入覆盖报告",[this]{fileAction("load_coverage","载入父任务覆盖报告","JSON (*.json)");});
    button(cr,1,0,"merge_coverage","合并补扫报告",[this]{fileAction("merge_coverage","选择补扫覆盖报告","JSON (*.json)");});
    coverage_=new QLabel("尚未审计；采集结束不等于覆盖完整");coverage_->setWordWrap(true);cl->addWidget(coverage_);
    table_=new QTableWidget(0,3);table_->setObjectName("coverageTable");table_->setHorizontalHeaderLabels({"轨道","未确认 / m","原因"});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);table_->setEditTriggers(QAbstractItemView::NoEditTriggers);table_->setMaximumHeight(145);table_->verticalHeader()->hide();table_->setWordWrap(true);cl->addWidget(table_);
    candidates_=new QComboBox;candidates_->setObjectName("rescanCandidates");cl->addWidget(candidates_);
    auto rescan=new QPushButton("预览选中补扫");rescan->setObjectName("rescan");buttons_["rescan"]=rescan;cl->addWidget(rescan);
    connect(rescan,&QPushButton::clicked,this,[this]{send("rescan",{{"index",candidates_->currentData().toInt()}});});
    layout->addStretch();timer_=new QTimer(this);connect(timer_,&QTimer::timeout,this,[this]{refresh();});timer_->start(250);refresh();
  }
  void load(const rviz_common::Config &config) override {
    Panel::load(config);config.mapGetBool("Preserve Dock Layout",&preserveLayout_);
  }
  void save(rviz_common::Config config) const override {
    Panel::save(config);config.mapSetValue("Preserve Dock Layout",true);
  }
  void onInitialize() override {
    if(!preserveLayout_)QTimer::singleShot(1000,this,[this]{
      auto dock=qobject_cast<QDockWidget*>(parentWidget());auto main=qobject_cast<QMainWindow*>(window());
      if(dock&&main){main->addDockWidget(Qt::RightDockWidgetArea,dock);main->resizeDocks({dock},{350},Qt::Horizontal);}
    });
    node_=getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    pub_=node_->create_publisher<std_msgs::msg::String>("/mission/operator/request",10);
    sub_=node_->create_subscription<std_msgs::msg::String>("/mission/operator/state",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){
      auto data=QByteArray::fromStdString(m->data);
      QMetaObject::invokeMethod(this,[this,data]{accept(QJsonDocument::fromJson(data).object());},Qt::QueuedConnection);
    });
  }
private:
  void addField(QFormLayout *form,const QString &key,const QString &label,double value,double min,double max,int decimals){
    auto s=new QDoubleSpinBox;s->setRange(min,max);s->setDecimals(decimals);s->setValue(value);s->setSingleStep(.1);s->setObjectName(key);numbers_[key]=s;form->addRow(label,s);
    connect(s,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this]{dirty_=true;refresh();});
  }
  template<class F> void button(QGridLayout *row,int y,int x,const QString &key,const QString &label,F action){auto b=new QPushButton(label);b->setObjectName(key);buttons_[key]=b;row->addWidget(b,y,x);connect(b,&QPushButton::clicked,this,action);}
  QJsonObject fields(){QJsonObject f{{"mission_id",id_->text()}};for(auto &p:numbers_)f[p.first]=p.second->value();f["speed"]=numbers_["speed"]->value()/3.6;return f;}
  void fileAction(const QString &action,const QString &title,const QString &filter){auto p=QFileDialog::getOpenFileName(this,title,QString(),filter);if(!p.isEmpty())send(action,{{"path",p}});}
  void send(const QString &action,QJsonObject value={}){
    if(!pub_)return;value["action"]=action;value["id"]=QUuid::createUuid().toString();
    std_msgs::msg::String m;m.data=QJsonDocument(value).toJson(QJsonDocument::Compact).toStdString();pub_->publish(m);
    if(action=="preview")previewId_=value["id"].toString();
    message_->setText("请求已发送…");
  }
  void accept(const QJsonObject &value){
    if(value.isEmpty())return;stateData_=value;heartbeat_.restart();
    numbers_["speed"]->setMaximum(value["max_requested_speed_m_s"].toDouble(10./3.6)*3.6);
    auto request=value["request"].toObject();auto encoded=QJsonDocument(request).toJson(QJsonDocument::Compact);
    if(encoded!=requestBytes_){
      requestBytes_=encoded;id_->setText(request["mission_id"].toString());auto r=request["region"].toObject();auto start=r["start_xy_m"].toArray();
      QJsonObject f{{"start_x",start[0]},{"start_y",start[1]},{"length",r["length_m"]},{"width",r["width_m"]},{"spacing",request["track_spacing_m"]},{"speed",request["scan_speed_m_s"].toDouble()*3.6},{"error",request["coverage_error_m"]}};
      for(auto &p:numbers_)p.second->setValue(f[p.first].toDouble());dirty_=false;
    }
    if(!previewId_.isEmpty() && value["response_id"].toString()==previewId_){if(value["ok"].toBool() && value["preview_valid"].toBool())dirty_=false;previewId_.clear();}
    message_->setText(value["message"].toString());message_->setStyleSheet(value["ok"].toBool()?"":"color: #e06b61;");
    auto c=value["coverage"].toObject();auto bytes=QJsonDocument(c).toJson(QJsonDocument::Compact);
    if(bytes!=coverageBytes_){coverageBytes_=bytes;showCoverage(c);}
    refresh();
  }
  void showCoverage(const QJsonObject &c){
    auto tracks=c["tracks"].toArray();table_->setRowCount(tracks.size());candidates_->clear();
    coverage_->setText(c.isEmpty()?"尚未审计；采集结束不等于覆盖完整":c["status"].toString()=="ESTIMATED_COMPLETE"?"估计覆盖完整（稀疏标签与误差预算范围内）":"存在未确认区域，见下表");
    for(int i=0;i<tracks.size();++i){auto t=tracks[i].toObject();QStringList gaps;for(auto v:t["unverified_along_m"].toArray()){auto a=v.toArray();gaps<<QString("%1–%2").arg(a[0].toDouble(),0,'f',2).arg(a[1].toDouble(),0,'f',2);}
      QStringList reasons;for(auto f:t["quality_flags"].toArray()){auto s=f.toString();reasons<<(s=="NO_CAPTURE_EVIDENCE"?"无采集证据":s=="UNCERTAINTY_EXCEEDS_OVERLAP"?"定位不确定度偏大":s=="FOOTPRINT_OUTSIDE_TARGET"?"足迹偏离":s);}
      table_->setItem(i,0,new QTableWidgetItem(QString::number(t["track_id"].toInt()+1)));table_->setItem(i,1,new QTableWidgetItem(gaps.isEmpty()?"无":gaps.join(", ")));table_->setItem(i,2,new QTableWidgetItem(reasons.join(" / ")));
    }
    table_->resizeRowsToContents();
    auto candidates=c["rescan_candidates"].toArray();for(int i=0;i<candidates.size();++i){auto v=candidates[i].toObject();if(v["status"].toString()=="PREVIEW_ONLY")candidates_->addItem(QString("轨道 %1 · 补扫 %2").arg(v["source_track_id"].toInt()+1).arg(i+1),i);}
  }
  QString imageDescription(const QJsonObject &image){
    if(image.isEmpty())return "本任务尚无原图";
    auto reason=image["end_reason"].toString();
    auto kind=reason=="full"?"满帧":reason=="capture_toggle"?"结束尾图":"异常尾图";
    return QString("最近原图：%1 × %2（%3）").arg(image["width"].toInt()).arg(image["rows"].toInt()).arg(kind);
  }
  void refresh(){
    bool online=heartbeat_.isValid()&&heartbeat_.elapsed()<2000;
    bool editable=online&&stateData_["editable"].toBool()&&!stateData_["busy"].toBool();
    connection_->setText(online?"● 巡检后端已连接":"○ 巡检后端未连接 / 状态超时");
    auto s=stateData_["status"].toObject();auto state=s["state"].toString("IDLE");
    static const std::map<QString,QString> names{{"IDLE","未准备"},{"READY","等待开始"},{"RUNNING","运行中"},{"PAUSING","制动暂停中"},{"PAUSED","已暂停"},{"CANCELING","取消停车中"},{"CANCELED","已取消"},{"ACQUIRED","采集结束"},{"COMPLETED","运动完成"},{"FAULT","故障锁存"}};
    auto it=names.find(state);state_->setText(QString("任务：%1  | 底盘：%2\n当前轨道：%3  | 相机：%4\n已归档：%6 张 · %7 行\n%8\n%5").arg(it==names.end()?state:it->second,s["motion_state"].toString("—"),s.contains("track_id")?QString::number(s["track_id"].toInt()+1):"—",s["capture_close_failed"].toBool()?"归档未确认，需重启采集":s["capture_sensor_enabled"].toBool()?"等待脉冲 / 采集":"关闭",s["reason"].toString()).arg(s["captured_blocks"].toInt()).arg(s["captured_rows"].toInt()).arg(imageDescription(s["last_image"].toObject())));
    progress_->setRange(0,std::max(1,s["step_count"].toInt()));progress_->setValue(s["step_index"].toInt());
    for(auto &p:numbers_)p.second->setEnabled(editable);id_->setEnabled(editable);
    for(auto &p:buttons_)p.second->setEnabled(editable);
    buttons_["prepare"]->setEnabled(editable&&!dirty_&&stateData_["preview_valid"].toBool());
    bool free=online&&!stateData_["busy"].toBool();
    buttons_["start"]->setEnabled(free&&state=="READY"&&s["ready_to_start"].toBool());
    buttons_["pause"]->setEnabled(free&&state=="RUNNING");buttons_["resume"]->setEnabled(free&&state=="PAUSED");
    buttons_["cancel"]->setEnabled(free&&(state=="RUNNING"||state=="PAUSED"||state=="PAUSING"||state=="READY"));
    buttons_["audit"]->setEnabled(editable&&(state=="ACQUIRED"||state=="CANCELED"||state=="FAULT"));
    buttons_["rescan"]->setEnabled(editable&&candidates_->count()>0);
    buttons_["merge_coverage"]->setEnabled(editable&&!stateData_["coverage"].toObject().isEmpty());
  }
  rclcpp::Node::SharedPtr node_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::map<QString,QDoubleSpinBox*> numbers_;std::map<QString,QPushButton*> buttons_;
  QLineEdit *id_;QLabel *connection_,*state_,*message_,*coverage_;QProgressBar *progress_;QTableWidget *table_;QComboBox *candidates_;QTimer *timer_;
  QElapsedTimer heartbeat_;QJsonObject stateData_;QByteArray requestBytes_,coverageBytes_;bool dirty_=true,preserveLayout_=false;QString previewId_;
};
}
PLUGINLIB_EXPORT_CLASS(agv_rviz::MissionPanel,rviz_common::Panel)
#include "mission_panel.moc"
