#include "../src/mission_panel.cpp"
#include <QtTest/QtTest>
namespace agv_rviz {
class MissionPanelTest : public QObject {
 Q_OBJECT
private Q_SLOTS:
 void controlsFollowBackendAndDirtyParameters(){
   MissionPanel p;
   QVERIFY(!p.buttons_["start"]->isEnabled());
   QJsonObject request{{"mission_id","test"},{"region",QJsonObject{{"start_xy_m",QJsonArray{6.,-1.}},{"length_m",3.},{"width_m",2.}}},{"track_spacing_m",1.},{"scan_speed_m_s",.5},{"coverage_error_m",.1}};
   QJsonObject v{{"request",request},{"editable",true},{"ok",true},{"preview_valid",true},{"status",QJsonObject{{"state","READY"},{"ready_to_start",true}}}};
   p.accept(v);QVERIFY(p.buttons_["start"]->isEnabled());QVERIFY(p.buttons_["prepare"]->isEnabled());
   p.numbers_["length"]->setValue(4.);QVERIFY(!p.buttons_["prepare"]->isEnabled());
   v["status"]=QJsonObject{{"state","RUNNING"}};v["editable"]=false;p.accept(v);
   QVERIFY(p.buttons_["pause"]->isEnabled());QVERIFY(p.buttons_["cancel"]->isEnabled());QVERIFY(!p.numbers_["length"]->isEnabled());QVERIFY(!p.buttons_["load"]->isEnabled());
   v["status"]=QJsonObject{{"state","PAUSED"}};p.accept(v);QVERIFY(p.buttons_["resume"]->isEnabled());QVERIFY(!p.buttons_["start"]->isEnabled());
   v["status"]=QJsonObject{{"state","FAULT"}};v["editable"]=true;p.accept(v);QVERIFY(p.buttons_["audit"]->isEnabled());QVERIFY(!p.buttons_["resume"]->isEnabled());
   p.heartbeat_.invalidate();p.refresh();QVERIFY(!p.buttons_["audit"]->isEnabled());QVERIFY(!p.buttons_["start"]->isEnabled());
 }
 void coverageDistinguishesUncertaintyFromMissingData(){
   MissionPanel p;QJsonObject c{{"status","NEEDS_RESCAN"},{"tracks",QJsonArray{
      QJsonObject{{"track_id",0},{"unverified_along_m",QJsonArray{QJsonArray{0.,3.}}},{"quality_flags",QJsonArray{"UNCERTAINTY_EXCEEDS_OVERLAP"}}},
      QJsonObject{{"track_id",1},{"unverified_along_m",QJsonArray{QJsonArray{0.,3.}}},{"quality_flags",QJsonArray{"NO_CAPTURE_EVIDENCE"}}}}}};
   p.showCoverage(c);QCOMPARE(p.table_->rowCount(),2);QVERIFY(p.table_->item(0,2)->text().contains("不确定度"));QVERIFY(p.table_->item(1,2)->text().contains("无采集"));
 }
};
}
QTEST_MAIN(agv_rviz::MissionPanelTest)
#include "test_panel.moc"
