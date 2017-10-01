using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ground_Station
{ 
    public partial class Ground_Station : Form
    {
        //Initial Declaration of Objects
        cMsgUdpT01 MsgUdpT01 = new cMsgUdpT01();
        cMsgUdpR01 MsgUdpR01 = new cMsgUdpR01();
        String textFileName = "";
        DateTime startTime=DateTime.Now;
        System.IO.FileStream F;
        UdpClient myUdpServer = new UdpClient(cConfig.UDP_PORT_NUM);
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, cConfig.UDP_PORT_NUM);
        bool clientConnected = false;
        
        cDataAnalysisClass DataAnalysisObj = new cDataAnalysisClass();
        cDataTxDisplayClass DataTxDisplayObj = new cDataTxDisplayClass();
        
        public Ground_Station()
        {
            vfnDeclaration();
        }        
        public void vfnDeclaration()
        {
            //Initialize Form
            InitializeComponent();

            bwUdpTransmit.WorkerReportsProgress = true;
            bwUdpTransmit.DoWork += new DoWorkEventHandler(bwUdpTransmit_DoWork);
                      

        }

        private void bwUdpTransmit_DoWork(object sender, DoWorkEventArgs e)
        {
            while (true)
            {
                MsgUdpT01.getPacket();

                try
                {
                    myUdpServer.Send(MsgUdpT01.dataBytes, Marshal.SizeOf(MsgUdpT01.message), remoteEP);
                    MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandNoAction);
                }
                catch
                {

                }
                System.Threading.Thread.Sleep(200);
            }

        }

        private void Ground_Station_Load(object sender, EventArgs e)
        {

            ssMainLabel1.Text = "IP:" + GetLocalIPv4();
            ssMainLabel2.Text = "Port#:" + cConfig.UDP_PORT_NUM.ToString();

            DataAnalysisObj.init(lvDataAnalysis, pnlDataAnalysis);
            DataTxDisplayObj.init(lvDataTx, MsgUdpT01.message);
            
            bwUdpTransmit.RunWorkerAsync();
            UDPListener();

            timerDisplayRefresh.Enabled = true;


            textFileName = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "_" + DateTime.Now.Hour.ToString() + "-" + DateTime.Now.Minute.ToString() + "-" + DateTime.Now.Second.ToString() + ".txt";


            String insertLine = "";
            foreach (var prop in DataAnalysisObj.data.GetType().GetProperties())
            {
                insertLine = insertLine + prop.Name + " ";

            }
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@textFileName, true))
            {
                file.WriteLine(insertLine);
            }

        }
        
        
        private void checkUdpClientStatus()
        {
            if (clientConnected)
            {
                ssMainLabel3.Text = "Connected";
                ssMainLabel4.Text = "Client IP:" + remoteEP.Address.ToString();
            }
            else
            {
                ssMainLabel3.Text = "Not Connected";
                ssMainLabel4.Text = "Client IP:" + "---";
            }
        }
        
        private void timerDisplayRefresh_Tick(object sender, EventArgs e)
        {

            DataAnalysisObj.update(lvDataAnalysis, pnlDataAnalysis, tbGraphBackColor.BackColor);
            DataTxDisplayObj.update(lvDataTx, MsgUdpT01.message);

            checkUdpClientStatus();
        }

        private void writeToTextFile()
        {
            String insertLine = " ";
            foreach (var prop in DataAnalysisObj.data.GetType().GetProperties())
            {
                insertLine = insertLine + string.Format("{0,4:0.00}", Convert.ToDouble(prop.GetValue(DataAnalysisObj.data, null).ToString())) + "  ";
            }

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@textFileName, true))
            {
                file.WriteLine(insertLine);
            }

        }

        private void updateMessagesForDataAnalysis()
        {
            MsgUdpR01.setPacket();

            DataAnalysisObj.data = MsgUdpR01.message;

        }

        private void button_GraphSelectBackColor_Click(object sender, EventArgs e)
        {
            colorDialog_Graph.ShowDialog();
            tbGraphBackColor.BackColor = colorDialog_Graph.Color;
        }

        private void button_GraphSelectColor_Click(object sender, EventArgs e)
        {
            colorDialog_Graph.ShowDialog();
            tbGraphLineColor.BackColor = colorDialog_Graph.Color;
        }

        private void button_GraphSetDefault_Click(object sender, EventArgs e)
        {
            trackBar_hOffset.Value = 0;
            trackBar_hOffset_Scroll(this, null);

            trackBar_vOffset.Value = 0;
            trackBar_vOffset_Scroll(this, null);

            numericUpDown_hScale.Value = (decimal)1.0;
            numericUpDown_hScale_ValueChanged(this, null);

            numericUpDown_vScale.Value = (decimal)1.0;
            numericUpDown_vScale_ValueChanged(this, null);
        }

        private void trackBar_vOffset_Scroll(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);

            textBox_trackBarGraphVer.Text = trackBar_vOffset.Value.ToString();
        }

        private void trackBar_hOffset_Scroll(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);

            textBox_trackBarGraphHor.Text = trackBar_hOffset.Value.ToString();
        }

        private void numericUpDown_hScale_ValueChanged(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);

        }

        private void numericUpDown_vScale_ValueChanged(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);
        }

        private void button_InsertGraph_Click(object sender, EventArgs e)
        {
            DataAnalysisObj.insertGraph(lvDataAnalysis, pnlDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value, tbGraphLineColor.BackColor);
        }

        private void button_RemoveGraph_Click(object sender, EventArgs e)
        {
            DataAnalysisObj.removeGraph(lvDataAnalysis);
        }

        private void textBox_trackBarGraphVer_TextChanged(object sender, EventArgs e)
        {
            try
            {
                trackBar_vOffset.Value = Convert.ToInt16(textBox_trackBarGraphVer.Text);
                trackBar_vOffset_Scroll(this, null);
                textBox_trackBarGraphVer.BackColor = Color.White;
            }
            catch
            {
                textBox_trackBarGraphVer.BackColor = Color.Red;

            }
        }

        private void textBox_trackBarGraphHor_TextChanged(object sender, EventArgs e)
        {
            try
            {
                trackBar_hOffset.Value = Convert.ToInt16(textBox_trackBarGraphHor.Text);
                trackBar_hOffset_Scroll(this, null);
                textBox_trackBarGraphHor.BackColor = Color.White;
            }
            catch
            {
                textBox_trackBarGraphHor.BackColor = Color.Red;

            }
        }
   
        private void btnDataAnalysisSelectAll_Click(object sender, EventArgs e)
        {
            foreach (ListViewItem item in lvDataAnalysis.Items)
            {
                item.Checked = true;
            }
        }

        private void btnDataAnalysisDeselectAll_Click(object sender, EventArgs e)
        {

            foreach (ListViewItem item in lvDataAnalysis.Items)
            {
                item.Checked = false;
            }
        }

        private void lvDataTx_SelectedIndexChanged(object sender, EventArgs e)
        {
        }

        private void lvDataTx_ItemChecked(object sender, ItemCheckedEventArgs e)
        {
            foreach (ListViewItem item in lvDataTx.Items)
            {
                if(item.Checked)
                {
                    numericUpDownDataTx.Value = Convert.ToDecimal(item.SubItems[1].Text);
                    break;
                }
            }

        }        

        private void numericUpDownDataTx_ValueChanged(object sender, EventArgs e)
        {
            if (DataTxDisplayObj.setData(lvDataTx, ref MsgUdpT01.message, Convert.ToDouble(numericUpDownDataTx.Value)))
            {
                numericUpDownDataTx.BackColor = Color.White;
            }
            else
            {
                numericUpDownDataTx.BackColor = Color.Red;
            }
        }        
        
        private void tb_pid_angle_pitch_roll_kp_Scroll(object sender, EventArgs e)
        {
            
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_kp, tb_pid_angle_pitch_roll_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);

            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
        }

        private void tb_pid_angle_pitch_roll_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_ki, tb_pid_angle_pitch_roll_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
        }

        private void tb_pid_angle_pitch_roll_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_kd, tb_pid_angle_pitch_roll_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
        }

        private void tb_pid_rate_pitch_roll_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_kp, tb_pid_rate_pitch_roll_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
        }

        private void tb_pid_rate_pitch_roll_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_ki, tb_pid_rate_pitch_roll_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
        }

        private void tb_pid_rate_pitch_roll_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_kd, tb_pid_rate_pitch_roll_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
        }
        
        private void tb_pid_angle_yaw_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_kp, tb_pid_angle_yaw_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
        }

        private void tb_pid_angle_yaw_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_ki, tb_pid_angle_yaw_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
        }

        private void tb_pid_angle_yaw_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_kd, tb_pid_angle_yaw_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
        }

        private void tb_pid_rate_yaw_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_kp, tb_pid_rate_yaw_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
        }

        private void tb_pid_rate_yaw_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_ki, tb_pid_rate_yaw_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
        }

        private void tb_pid_rate_yaw_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_kd, tb_pid_rate_yaw_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
        }

        private void tb_pid_acc_alt_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_acc_alt_kp, tb_pid_acc_alt_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
        }

        private void tb_pid_acc_alt_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_acc_alt_ki, tb_pid_acc_alt_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
        }

        private void tb_pid_acc_alt_kd_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_alt_kd, tb_pid_acc_alt_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
        }
        
        private void tb_pid_vel_alt_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_vel_alt_kp, tb_pid_vel_alt_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
        }

        private void tb_pid_vel_alt_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_vel_alt_ki, tb_pid_vel_alt_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
        }

        private void tb_pid_vel_alt_kd_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_vel_alt_kd, tb_pid_vel_alt_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
        }
        
        private void cb_AltitudeHold_CheckedChanged(object sender, EventArgs e)
        {
            if(cb_AltitudeHold.Checked)
            {
                MsgUdpT01.message.autoModeCommand = Convert.ToByte(autoModeType.autoModeAltitude);
            }
            else
            {
                MsgUdpT01.message.autoModeCommand = Convert.ToByte(autoModeType.autoModeOFF);
            }
        }

        private void button_setallcmd_Click(object sender, EventArgs e)
        {

            tb_pid_angle_pitch_roll_kp_Scroll(this, e);
            tb_pid_rate_pitch_roll_kp_Scroll(this, e);
            tb_pid_angle_yaw_kp_Scroll(this, e);
            tb_pid_rate_yaw_kp_Scroll(this, e);
            tb_pid_vel_alt_kp_Scroll(this, e);
            tb_pid_acc_alt_kp_Scroll(this, e);
            tb_pid_pos_alt_kp_Scroll(this, e);
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAll);
        }        

        private void UDPListener()
        {
            Task.Run(async () =>
            {
                using (myUdpServer)
                {
                    while (true)
                    {
                        var receivedResults = await myUdpServer.ReceiveAsync();
                        MsgUdpR01.dataBytes = receivedResults.Buffer;
                        remoteEP = receivedResults.RemoteEndPoint;
                        updateMessagesForDataAnalysis();
                        writeToTextFile();
                        clientConnected = true;
                    }
                }
            });
        }

        public string GetLocalIPv4()
        {
            NetworkInterfaceType _type = NetworkInterfaceType.Wireless80211;
            string output = "";
            foreach (NetworkInterface item in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (item.NetworkInterfaceType == _type && item.OperationalStatus == OperationalStatus.Up)
                {
                    IPInterfaceProperties adapterProperties = item.GetIPProperties();

                    if (adapterProperties.GatewayAddresses.FirstOrDefault() != null)
                    {
                        foreach (UnicastIPAddressInformation ip in adapterProperties.UnicastAddresses)
                        {
                            if (ip.Address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                            {
                                output = ip.Address.ToString();
                            }
                        }
                    }
                }
            }
            return output;
        }

        private void tb_pid_pos_alt_kp_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_pos_alt_kp, tb_pid_pos_alt_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyPosAlt);

            MsgUdpT01.message.pidPosAltKp = Convert.ToByte(tb_pid_pos_alt_kp.Value);
            MsgUdpT01.message.pidPosAltKi = Convert.ToByte(tb_pid_pos_alt_ki.Value);
            MsgUdpT01.message.pidPosAltKd = Convert.ToByte(tb_pid_pos_alt_kd.Value);
        }

        private void tb_pid_pos_alt_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_pos_alt_ki, tb_pid_pos_alt_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyPosAlt);

            MsgUdpT01.message.pidPosAltKp = Convert.ToByte(tb_pid_pos_alt_kp.Value);
            MsgUdpT01.message.pidPosAltKi = Convert.ToByte(tb_pid_pos_alt_ki.Value);
            MsgUdpT01.message.pidPosAltKd = Convert.ToByte(tb_pid_pos_alt_kd.Value);
        }

        private void tb_pid_pos_alt_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_pos_alt_kd, tb_pid_pos_alt_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyPosAlt);

            MsgUdpT01.message.pidPosAltKp = Convert.ToByte(tb_pid_pos_alt_kp.Value);
            MsgUdpT01.message.pidPosAltKi = Convert.ToByte(tb_pid_pos_alt_ki.Value);
            MsgUdpT01.message.pidPosAltKd = Convert.ToByte(tb_pid_pos_alt_kd.Value);
        }

        private void tb_pid_acc_posxy_kp_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_posxy_kp, tb_pid_acc_posxy_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccPos);

            MsgUdpT01.message.pidAccPosKp = Convert.ToByte(tb_pid_acc_posxy_kp.Value);
            MsgUdpT01.message.pidAccPosKi = Convert.ToByte(tb_pid_acc_posxy_ki.Value);
            MsgUdpT01.message.pidAccPosKd = Convert.ToByte(tb_pid_acc_posxy_kd.Value);
        }

        private void tb_pid_acc_posxy_ki_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_posxy_ki, tb_pid_acc_posxy_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccPos);

            MsgUdpT01.message.pidAccPosKp = Convert.ToByte(tb_pid_acc_posxy_kp.Value);
            MsgUdpT01.message.pidAccPosKi = Convert.ToByte(tb_pid_acc_posxy_ki.Value);
            MsgUdpT01.message.pidAccPosKd = Convert.ToByte(tb_pid_acc_posxy_kd.Value);
        }

        private void tb_pid_acc_posxy_kd_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_posxy_kd, tb_pid_acc_posxy_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccPos);

            MsgUdpT01.message.pidAccPosKp = Convert.ToByte(tb_pid_acc_posxy_kp.Value);
            MsgUdpT01.message.pidAccPosKi = Convert.ToByte(tb_pid_acc_posxy_ki.Value);
            MsgUdpT01.message.pidAccPosKd = Convert.ToByte(tb_pid_acc_posxy_kd.Value);
        }

        private void cb_saveHome_CheckedChanged(object sender, EventArgs e)
        {
            if (cb_saveHome.Checked)
                MsgUdpT01.message.saveHomePos = 0xAA;  //decimal 170
            else
                MsgUdpT01.message.saveHomePos = 0x00; // decimal 0
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            F.Read(MsgUdpR01.dataBytes, 0, Marshal.SizeOf(MsgUdpR01.message));

            trackBar_record.Value = Convert.ToInt32(F.Position / Marshal.SizeOf(MsgUdpR01.message));
            if(cb_record_display.Checked)  updateMessagesForDataAnalysis();
            if(cb_record_save.Checked) writeToTextFile();
        }

        private void button1_Click(object sender, EventArgs e)
        {
           
            if(ofd_record.ShowDialog() == DialogResult.OK)
            {
                label_record.Text = ofd_record.FileName;

                try
                {
                    F = new FileStream(ofd_record.FileName, FileMode.Open, FileAccess.Read, FileShare.Read);
                    trackBar_record.Maximum = Convert.ToInt32(F.Length / Marshal.SizeOf(MsgUdpR01.message));
                    label_packet_size.Text = "Packet Size:" + trackBar_record.Maximum.ToString();
                }
                catch
                {

                }
            }


        }

        private void b_play_Click(object sender, EventArgs e)
        {
            if(b_play.Text == "Play")
            {

                timer_record.Enabled = true;
                b_play.Text = "Pause";
            }
            else
            {
                timer_record.Enabled = false;
                b_play.Text = "Play";

                F.Position = 0;
            }

        }

        private void trackBar_record_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(trackBar_record, trackBar_record.Value.ToString());
            F.Position = trackBar_record.Value * Marshal.SizeOf(MsgUdpR01.message);
        }
    }
}

public static class ControlExtensions
{
    public static void DoubleBuffered(this Control control, bool enable)
    {
        var doubleBufferPropertyInfo = control.GetType().GetProperty("DoubleBuffered", BindingFlags.Instance | BindingFlags.NonPublic);
        doubleBufferPropertyInfo.SetValue(control, enable, null);
    }
}


