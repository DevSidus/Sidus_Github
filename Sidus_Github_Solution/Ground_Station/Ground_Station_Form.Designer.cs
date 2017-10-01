namespace Ground_Station
{
    partial class Ground_Station
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.pnlTopPanel = new System.Windows.Forms.Panel();
            this.pnlHeartBeat = new System.Windows.Forms.Panel();
            this.pnlBottomPanel = new System.Windows.Forms.Panel();
            this.ssMain = new System.Windows.Forms.StatusStrip();
            this.prbMainProgress = new System.Windows.Forms.ToolStripProgressBar();
            this.ssMainLabel1 = new System.Windows.Forms.ToolStripStatusLabel();
            this.ssMainLabel2 = new System.Windows.Forms.ToolStripStatusLabel();
            this.ssMainLabel3 = new System.Windows.Forms.ToolStripStatusLabel();
            this.ssMainLabel4 = new System.Windows.Forms.ToolStripStatusLabel();
            this.timerDisplayRefresh = new System.Windows.Forms.Timer(this.components);
            this.colorDialog_Graph = new System.Windows.Forms.ColorDialog();
            this.pnlContentPanel = new System.Windows.Forms.Panel();
            this.tableLayoutContent = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanelDataAnalysis = new System.Windows.Forms.TableLayoutPanel();
            this.lvDataAnalysis = new System.Windows.Forms.ListView();
            this.columnVariableName = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnValue = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.tableLayoutPanel2 = new System.Windows.Forms.TableLayoutPanel();
            this.pnlDataAnalysis = new System.Windows.Forms.Panel();
            this.tableLayoutPanel4 = new System.Windows.Forms.TableLayoutPanel();
            this.textBox_trackBarGraphHor = new System.Windows.Forms.TextBox();
            this.trackBar_hOffset = new System.Windows.Forms.TrackBar();
            this.tableLayoutPanel6 = new System.Windows.Forms.TableLayoutPanel();
            this.textBox_trackBarGraphVer = new System.Windows.Forms.TextBox();
            this.trackBar_vOffset = new System.Windows.Forms.TrackBar();
            this.tableLayoutPanel3 = new System.Windows.Forms.TableLayoutPanel();
            this.btnDataAnalysisDeselectAll = new System.Windows.Forms.Button();
            this.tableLayoutPanel5 = new System.Windows.Forms.TableLayoutPanel();
            this.button_GraphSetDefault = new System.Windows.Forms.Button();
            this.button_RemoveGraph = new System.Windows.Forms.Button();
            this.button_InsertGraph = new System.Windows.Forms.Button();
            this.button_GraphSelectBackColor = new System.Windows.Forms.Button();
            this.button_GraphSelectColor = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.numericUpDown_vScale = new System.Windows.Forms.NumericUpDown();
            this.tbGraphBackColor = new System.Windows.Forms.TextBox();
            this.tbGraphLineColor = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.btnDataAnalysisSelectAll = new System.Windows.Forms.Button();
            this.numericUpDown_hScale = new System.Windows.Forms.NumericUpDown();
            this.cb_AltitudeHold = new System.Windows.Forms.CheckBox();
            this.button_setallcmd = new System.Windows.Forms.Button();
            this.tableLayoutPanelDataTx = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel7 = new System.Windows.Forms.TableLayoutPanel();
            this.tableLayoutPanel9 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_rate_pitch_roll_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_rate_pitch_roll_ki = new System.Windows.Forms.TrackBar();
            this.label8 = new System.Windows.Forms.Label();
            this.tb_pid_rate_pitch_roll_kd = new System.Windows.Forms.TrackBar();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.tableLayoutPanel8 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_angle_pitch_roll_kp = new System.Windows.Forms.TrackBar();
            this.label3 = new System.Windows.Forms.Label();
            this.tb_pid_angle_pitch_roll_ki = new System.Windows.Forms.TrackBar();
            this.label4 = new System.Windows.Forms.Label();
            this.tb_pid_angle_pitch_roll_kd = new System.Windows.Forms.TrackBar();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.tableLayoutPanel11 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_rate_yaw_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_rate_yaw_ki = new System.Windows.Forms.TrackBar();
            this.label15 = new System.Windows.Forms.Label();
            this.tb_pid_rate_yaw_kd = new System.Windows.Forms.TrackBar();
            this.label16 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.tableLayoutPanel12 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_angle_yaw_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_angle_yaw_ki = new System.Windows.Forms.TrackBar();
            this.label21 = new System.Windows.Forms.Label();
            this.tb_pid_angle_yaw_kd = new System.Windows.Forms.TrackBar();
            this.label22 = new System.Windows.Forms.Label();
            this.label23 = new System.Windows.Forms.Label();
            this.label24 = new System.Windows.Forms.Label();
            this.tableLayoutPanel13 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_pos_alt_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_pos_alt_ki = new System.Windows.Forms.TrackBar();
            this.label11 = new System.Windows.Forms.Label();
            this.tb_pid_pos_alt_kd = new System.Windows.Forms.TrackBar();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.tableLayoutPanel15 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_vel_alt_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_vel_alt_ki = new System.Windows.Forms.TrackBar();
            this.label35 = new System.Windows.Forms.Label();
            this.tb_pid_vel_alt_kd = new System.Windows.Forms.TrackBar();
            this.label36 = new System.Windows.Forms.Label();
            this.label37 = new System.Windows.Forms.Label();
            this.label38 = new System.Windows.Forms.Label();
            this.tableLayoutPanel14 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_acc_alt_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_acc_alt_ki = new System.Windows.Forms.TrackBar();
            this.label29 = new System.Windows.Forms.Label();
            this.tb_pid_acc_alt_kd = new System.Windows.Forms.TrackBar();
            this.label30 = new System.Windows.Forms.Label();
            this.label31 = new System.Windows.Forms.Label();
            this.label32 = new System.Windows.Forms.Label();
            this.tableLayoutPanel16 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_acc_posxy_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_acc_posxy_ki = new System.Windows.Forms.TrackBar();
            this.label19 = new System.Windows.Forms.Label();
            this.tb_pid_acc_posxy_kd = new System.Windows.Forms.TrackBar();
            this.label20 = new System.Windows.Forms.Label();
            this.label25 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.cb_saveHome = new System.Windows.Forms.CheckBox();
            this.tableLayoutPanel10 = new System.Windows.Forms.TableLayoutPanel();
            this.lvDataTx = new System.Windows.Forms.ListView();
            this.columnHeader1 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader2 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.numericUpDownDataTx = new System.Windows.Forms.NumericUpDown();
            this.tableLayoutPanel17 = new System.Windows.Forms.TableLayoutPanel();
            this.label_packet_size = new System.Windows.Forms.Label();
            this.b_open_file = new System.Windows.Forms.Button();
            this.label_record = new System.Windows.Forms.Label();
            this.b_play = new System.Windows.Forms.Button();
            this.trackBar_record = new System.Windows.Forms.TrackBar();
            this.tableLayoutPanel18 = new System.Windows.Forms.TableLayoutPanel();
            this.cb_record_save = new System.Windows.Forms.CheckBox();
            this.cb_record_display = new System.Windows.Forms.CheckBox();
            this.bwUdpTransmit = new System.ComponentModel.BackgroundWorker();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.timer_record = new System.Windows.Forms.Timer(this.components);
            this.ofd_record = new System.Windows.Forms.OpenFileDialog();
            this.pnlTopPanel.SuspendLayout();
            this.pnlBottomPanel.SuspendLayout();
            this.ssMain.SuspendLayout();
            this.pnlContentPanel.SuspendLayout();
            this.tableLayoutContent.SuspendLayout();
            this.tableLayoutPanelDataAnalysis.SuspendLayout();
            this.tableLayoutPanel2.SuspendLayout();
            this.tableLayoutPanel4.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_hOffset)).BeginInit();
            this.tableLayoutPanel6.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_vOffset)).BeginInit();
            this.tableLayoutPanel3.SuspendLayout();
            this.tableLayoutPanel5.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_vScale)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_hScale)).BeginInit();
            this.tableLayoutPanelDataTx.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.tableLayoutPanel7.SuspendLayout();
            this.tableLayoutPanel9.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kd)).BeginInit();
            this.tableLayoutPanel8.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kd)).BeginInit();
            this.tableLayoutPanel11.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kd)).BeginInit();
            this.tableLayoutPanel12.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kd)).BeginInit();
            this.tableLayoutPanel13.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_pos_alt_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_pos_alt_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_pos_alt_kd)).BeginInit();
            this.tableLayoutPanel15.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_vel_alt_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_vel_alt_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_vel_alt_kd)).BeginInit();
            this.tableLayoutPanel14.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_alt_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_alt_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_alt_kd)).BeginInit();
            this.tableLayoutPanel16.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_posxy_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_posxy_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_posxy_kd)).BeginInit();
            this.tableLayoutPanel10.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownDataTx)).BeginInit();
            this.tableLayoutPanel17.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_record)).BeginInit();
            this.tableLayoutPanel18.SuspendLayout();
            this.SuspendLayout();
            // 
            // pnlTopPanel
            // 
            this.pnlTopPanel.Controls.Add(this.pnlHeartBeat);
            this.pnlTopPanel.Dock = System.Windows.Forms.DockStyle.Top;
            this.pnlTopPanel.Location = new System.Drawing.Point(0, 0);
            this.pnlTopPanel.Name = "pnlTopPanel";
            this.pnlTopPanel.Size = new System.Drawing.Size(1484, 10);
            this.pnlTopPanel.TabIndex = 0;
            // 
            // pnlHeartBeat
            // 
            this.pnlHeartBeat.BackColor = System.Drawing.Color.OrangeRed;
            this.pnlHeartBeat.Dock = System.Windows.Forms.DockStyle.Top;
            this.pnlHeartBeat.Location = new System.Drawing.Point(0, 0);
            this.pnlHeartBeat.Name = "pnlHeartBeat";
            this.pnlHeartBeat.Size = new System.Drawing.Size(1484, 10);
            this.pnlHeartBeat.TabIndex = 0;
            // 
            // pnlBottomPanel
            // 
            this.pnlBottomPanel.Controls.Add(this.ssMain);
            this.pnlBottomPanel.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.pnlBottomPanel.Location = new System.Drawing.Point(0, 963);
            this.pnlBottomPanel.Name = "pnlBottomPanel";
            this.pnlBottomPanel.Size = new System.Drawing.Size(1484, 32);
            this.pnlBottomPanel.TabIndex = 1;
            // 
            // ssMain
            // 
            this.ssMain.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.prbMainProgress,
            this.ssMainLabel1,
            this.ssMainLabel2,
            this.ssMainLabel3,
            this.ssMainLabel4});
            this.ssMain.Location = new System.Drawing.Point(0, 10);
            this.ssMain.Name = "ssMain";
            this.ssMain.Size = new System.Drawing.Size(1484, 22);
            this.ssMain.TabIndex = 0;
            this.ssMain.Text = "statusStrip1";
            // 
            // prbMainProgress
            // 
            this.prbMainProgress.Name = "prbMainProgress";
            this.prbMainProgress.Size = new System.Drawing.Size(100, 16);
            // 
            // ssMainLabel1
            // 
            this.ssMainLabel1.Name = "ssMainLabel1";
            this.ssMainLabel1.Size = new System.Drawing.Size(92, 17);
            this.ssMainLabel1.Text = "IP: ???.???.???.???";
            // 
            // ssMainLabel2
            // 
            this.ssMainLabel2.Name = "ssMainLabel2";
            this.ssMainLabel2.Size = new System.Drawing.Size(59, 17);
            this.ssMainLabel2.Text = "Port#:????";
            // 
            // ssMainLabel3
            // 
            this.ssMainLabel3.Name = "ssMainLabel3";
            this.ssMainLabel3.Size = new System.Drawing.Size(88, 17);
            this.ssMainLabel3.Text = "Not Connected";
            // 
            // ssMainLabel4
            // 
            this.ssMainLabel4.Name = "ssMainLabel4";
            this.ssMainLabel4.Size = new System.Drawing.Size(123, 17);
            this.ssMainLabel4.Text = "Client IP:???.???.???.???";
            // 
            // timerDisplayRefresh
            // 
            this.timerDisplayRefresh.Interval = 50;
            this.timerDisplayRefresh.Tick += new System.EventHandler(this.timerDisplayRefresh_Tick);
            // 
            // pnlContentPanel
            // 
            this.pnlContentPanel.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.pnlContentPanel.Controls.Add(this.tableLayoutContent);
            this.pnlContentPanel.Location = new System.Drawing.Point(0, 12);
            this.pnlContentPanel.Name = "pnlContentPanel";
            this.pnlContentPanel.Size = new System.Drawing.Size(1484, 958);
            this.pnlContentPanel.TabIndex = 2;
            // 
            // tableLayoutContent
            // 
            this.tableLayoutContent.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutContent.ColumnCount = 1;
            this.tableLayoutContent.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutContent.Controls.Add(this.tableLayoutPanelDataAnalysis, 0, 0);
            this.tableLayoutContent.Controls.Add(this.tableLayoutPanelDataTx, 0, 1);
            this.tableLayoutContent.Controls.Add(this.tableLayoutPanel17, 0, 2);
            this.tableLayoutContent.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutContent.Name = "tableLayoutContent";
            this.tableLayoutContent.RowCount = 3;
            this.tableLayoutContent.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutContent.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutContent.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutContent.Size = new System.Drawing.Size(1484, 958);
            this.tableLayoutContent.TabIndex = 0;
            // 
            // tableLayoutPanelDataAnalysis
            // 
            this.tableLayoutPanelDataAnalysis.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanelDataAnalysis.ColumnCount = 2;
            this.tableLayoutPanelDataAnalysis.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 373F));
            this.tableLayoutPanelDataAnalysis.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 1168F));
            this.tableLayoutPanelDataAnalysis.Controls.Add(this.lvDataAnalysis, 0, 0);
            this.tableLayoutPanelDataAnalysis.Controls.Add(this.tableLayoutPanel2, 1, 0);
            this.tableLayoutPanelDataAnalysis.Location = new System.Drawing.Point(3, 3);
            this.tableLayoutPanelDataAnalysis.Name = "tableLayoutPanelDataAnalysis";
            this.tableLayoutPanelDataAnalysis.RowCount = 1;
            this.tableLayoutPanelDataAnalysis.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanelDataAnalysis.Size = new System.Drawing.Size(1478, 525);
            this.tableLayoutPanelDataAnalysis.TabIndex = 1;
            // 
            // lvDataAnalysis
            // 
            this.lvDataAnalysis.CheckBoxes = true;
            this.lvDataAnalysis.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnVariableName,
            this.columnValue});
            this.lvDataAnalysis.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.lvDataAnalysis.FullRowSelect = true;
            this.lvDataAnalysis.Location = new System.Drawing.Point(3, 3);
            this.lvDataAnalysis.Name = "lvDataAnalysis";
            this.lvDataAnalysis.Size = new System.Drawing.Size(367, 519);
            this.lvDataAnalysis.TabIndex = 0;
            this.lvDataAnalysis.UseCompatibleStateImageBehavior = false;
            this.lvDataAnalysis.View = System.Windows.Forms.View.Details;
            // 
            // columnVariableName
            // 
            this.columnVariableName.Text = "Variable Name";
            this.columnVariableName.Width = 224;
            // 
            // columnValue
            // 
            this.columnValue.Text = "Value";
            this.columnValue.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.columnValue.Width = 100;
            // 
            // tableLayoutPanel2
            // 
            this.tableLayoutPanel2.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanel2.ColumnCount = 3;
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel2.Controls.Add(this.pnlDataAnalysis, 1, 0);
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel4, 1, 1);
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel6, 2, 0);
            this.tableLayoutPanel2.Controls.Add(this.tableLayoutPanel3, 1, 2);
            this.tableLayoutPanel2.Location = new System.Drawing.Point(376, 3);
            this.tableLayoutPanel2.Name = "tableLayoutPanel2";
            this.tableLayoutPanel2.RowCount = 3;
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 92.27723F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 7.722772F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 72F));
            this.tableLayoutPanel2.Size = new System.Drawing.Size(1162, 519);
            this.tableLayoutPanel2.TabIndex = 1;
            // 
            // pnlDataAnalysis
            // 
            this.pnlDataAnalysis.Location = new System.Drawing.Point(3, 3);
            this.pnlDataAnalysis.Name = "pnlDataAnalysis";
            this.pnlDataAnalysis.Size = new System.Drawing.Size(1047, 406);
            this.pnlDataAnalysis.TabIndex = 6;
            // 
            // tableLayoutPanel4
            // 
            this.tableLayoutPanel4.ColumnCount = 2;
            this.tableLayoutPanel4.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 94.33962F));
            this.tableLayoutPanel4.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 5.660378F));
            this.tableLayoutPanel4.Controls.Add(this.textBox_trackBarGraphHor, 0, 0);
            this.tableLayoutPanel4.Controls.Add(this.trackBar_hOffset, 0, 0);
            this.tableLayoutPanel4.Location = new System.Drawing.Point(3, 415);
            this.tableLayoutPanel4.Name = "tableLayoutPanel4";
            this.tableLayoutPanel4.RowCount = 1;
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel4.Size = new System.Drawing.Size(1047, 28);
            this.tableLayoutPanel4.TabIndex = 8;
            // 
            // textBox_trackBarGraphHor
            // 
            this.textBox_trackBarGraphHor.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.textBox_trackBarGraphHor.Location = new System.Drawing.Point(990, 3);
            this.textBox_trackBarGraphHor.Name = "textBox_trackBarGraphHor";
            this.textBox_trackBarGraphHor.Size = new System.Drawing.Size(54, 20);
            this.textBox_trackBarGraphHor.TabIndex = 13;
            this.textBox_trackBarGraphHor.Text = "0";
            this.textBox_trackBarGraphHor.TextChanged += new System.EventHandler(this.textBox_trackBarGraphHor_TextChanged);
            // 
            // trackBar_hOffset
            // 
            this.trackBar_hOffset.Location = new System.Drawing.Point(3, 3);
            this.trackBar_hOffset.Maximum = 400;
            this.trackBar_hOffset.Minimum = -400;
            this.trackBar_hOffset.Name = "trackBar_hOffset";
            this.trackBar_hOffset.Size = new System.Drawing.Size(981, 22);
            this.trackBar_hOffset.TabIndex = 6;
            this.trackBar_hOffset.Scroll += new System.EventHandler(this.trackBar_hOffset_Scroll);
            // 
            // tableLayoutPanel6
            // 
            this.tableLayoutPanel6.ColumnCount = 1;
            this.tableLayoutPanel6.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel6.Controls.Add(this.textBox_trackBarGraphVer, 0, 1);
            this.tableLayoutPanel6.Controls.Add(this.trackBar_vOffset, 0, 0);
            this.tableLayoutPanel6.Location = new System.Drawing.Point(1056, 3);
            this.tableLayoutPanel6.Name = "tableLayoutPanel6";
            this.tableLayoutPanel6.RowCount = 2;
            this.tableLayoutPanel6.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel6.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 24F));
            this.tableLayoutPanel6.Size = new System.Drawing.Size(45, 406);
            this.tableLayoutPanel6.TabIndex = 9;
            // 
            // textBox_trackBarGraphVer
            // 
            this.textBox_trackBarGraphVer.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.textBox_trackBarGraphVer.Location = new System.Drawing.Point(3, 385);
            this.textBox_trackBarGraphVer.Name = "textBox_trackBarGraphVer";
            this.textBox_trackBarGraphVer.Size = new System.Drawing.Size(35, 20);
            this.textBox_trackBarGraphVer.TabIndex = 12;
            this.textBox_trackBarGraphVer.Text = "0";
            this.textBox_trackBarGraphVer.TextChanged += new System.EventHandler(this.textBox_trackBarGraphVer_TextChanged);
            // 
            // trackBar_vOffset
            // 
            this.trackBar_vOffset.Location = new System.Drawing.Point(3, 3);
            this.trackBar_vOffset.Maximum = 1200;
            this.trackBar_vOffset.Minimum = -1200;
            this.trackBar_vOffset.Name = "trackBar_vOffset";
            this.trackBar_vOffset.Orientation = System.Windows.Forms.Orientation.Vertical;
            this.trackBar_vOffset.Size = new System.Drawing.Size(39, 376);
            this.trackBar_vOffset.TabIndex = 7;
            this.trackBar_vOffset.Scroll += new System.EventHandler(this.trackBar_vOffset_Scroll);
            // 
            // tableLayoutPanel3
            // 
            this.tableLayoutPanel3.ColumnCount = 7;
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 79F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 109F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 39F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 60F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.26531F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 86.7347F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 227F));
            this.tableLayoutPanel3.Controls.Add(this.btnDataAnalysisDeselectAll, 0, 1);
            this.tableLayoutPanel3.Controls.Add(this.tableLayoutPanel5, 5, 0);
            this.tableLayoutPanel3.Controls.Add(this.button_GraphSelectBackColor, 1, 0);
            this.tableLayoutPanel3.Controls.Add(this.button_GraphSelectColor, 1, 1);
            this.tableLayoutPanel3.Controls.Add(this.label2, 3, 1);
            this.tableLayoutPanel3.Controls.Add(this.numericUpDown_vScale, 4, 1);
            this.tableLayoutPanel3.Controls.Add(this.tbGraphBackColor, 2, 0);
            this.tableLayoutPanel3.Controls.Add(this.tbGraphLineColor, 2, 1);
            this.tableLayoutPanel3.Controls.Add(this.label1, 3, 0);
            this.tableLayoutPanel3.Controls.Add(this.btnDataAnalysisSelectAll, 0, 0);
            this.tableLayoutPanel3.Controls.Add(this.numericUpDown_hScale, 4, 0);
            this.tableLayoutPanel3.Controls.Add(this.cb_AltitudeHold, 6, 0);
            this.tableLayoutPanel3.Controls.Add(this.button_setallcmd, 6, 1);
            this.tableLayoutPanel3.Location = new System.Drawing.Point(3, 449);
            this.tableLayoutPanel3.Name = "tableLayoutPanel3";
            this.tableLayoutPanel3.RowCount = 2;
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel3.Size = new System.Drawing.Size(1047, 66);
            this.tableLayoutPanel3.TabIndex = 7;
            // 
            // btnDataAnalysisDeselectAll
            // 
            this.btnDataAnalysisDeselectAll.Location = new System.Drawing.Point(3, 36);
            this.btnDataAnalysisDeselectAll.Name = "btnDataAnalysisDeselectAll";
            this.btnDataAnalysisDeselectAll.Size = new System.Drawing.Size(73, 27);
            this.btnDataAnalysisDeselectAll.TabIndex = 27;
            this.btnDataAnalysisDeselectAll.Text = "Deselect All";
            this.btnDataAnalysisDeselectAll.UseVisualStyleBackColor = true;
            this.btnDataAnalysisDeselectAll.Click += new System.EventHandler(this.btnDataAnalysisDeselectAll_Click);
            // 
            // tableLayoutPanel5
            // 
            this.tableLayoutPanel5.ColumnCount = 3;
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 48.91304F));
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 51.08696F));
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 254F));
            this.tableLayoutPanel5.Controls.Add(this.button_GraphSetDefault, 0, 0);
            this.tableLayoutPanel5.Controls.Add(this.button_RemoveGraph, 2, 0);
            this.tableLayoutPanel5.Controls.Add(this.button_InsertGraph, 1, 0);
            this.tableLayoutPanel5.Location = new System.Drawing.Point(360, 3);
            this.tableLayoutPanel5.Name = "tableLayoutPanel5";
            this.tableLayoutPanel5.RowCount = 1;
            this.tableLayoutPanel5.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel5.Size = new System.Drawing.Size(456, 27);
            this.tableLayoutPanel5.TabIndex = 25;
            // 
            // button_GraphSetDefault
            // 
            this.button_GraphSetDefault.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.button_GraphSetDefault.Location = new System.Drawing.Point(3, 3);
            this.button_GraphSetDefault.Name = "button_GraphSetDefault";
            this.button_GraphSetDefault.Size = new System.Drawing.Size(92, 21);
            this.button_GraphSetDefault.TabIndex = 11;
            this.button_GraphSetDefault.Text = "Set Default";
            this.button_GraphSetDefault.UseVisualStyleBackColor = true;
            this.button_GraphSetDefault.Click += new System.EventHandler(this.button_GraphSetDefault_Click);
            // 
            // button_RemoveGraph
            // 
            this.button_RemoveGraph.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.button_RemoveGraph.Location = new System.Drawing.Point(204, 3);
            this.button_RemoveGraph.Name = "button_RemoveGraph";
            this.button_RemoveGraph.Size = new System.Drawing.Size(249, 21);
            this.button_RemoveGraph.TabIndex = 13;
            this.button_RemoveGraph.Text = "Remove";
            this.button_RemoveGraph.UseVisualStyleBackColor = true;
            this.button_RemoveGraph.Click += new System.EventHandler(this.button_RemoveGraph_Click);
            // 
            // button_InsertGraph
            // 
            this.button_InsertGraph.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.button_InsertGraph.Location = new System.Drawing.Point(101, 3);
            this.button_InsertGraph.Name = "button_InsertGraph";
            this.button_InsertGraph.Size = new System.Drawing.Size(97, 21);
            this.button_InsertGraph.TabIndex = 12;
            this.button_InsertGraph.Text = "Insert";
            this.button_InsertGraph.UseVisualStyleBackColor = true;
            this.button_InsertGraph.Click += new System.EventHandler(this.button_InsertGraph_Click);
            // 
            // button_GraphSelectBackColor
            // 
            this.button_GraphSelectBackColor.Location = new System.Drawing.Point(82, 3);
            this.button_GraphSelectBackColor.Name = "button_GraphSelectBackColor";
            this.button_GraphSelectBackColor.Size = new System.Drawing.Size(103, 27);
            this.button_GraphSelectBackColor.TabIndex = 20;
            this.button_GraphSelectBackColor.Text = "Select Back Color";
            this.button_GraphSelectBackColor.UseVisualStyleBackColor = true;
            this.button_GraphSelectBackColor.Click += new System.EventHandler(this.button_GraphSelectBackColor_Click);
            // 
            // button_GraphSelectColor
            // 
            this.button_GraphSelectColor.Location = new System.Drawing.Point(82, 36);
            this.button_GraphSelectColor.Name = "button_GraphSelectColor";
            this.button_GraphSelectColor.Size = new System.Drawing.Size(103, 27);
            this.button_GraphSelectColor.TabIndex = 19;
            this.button_GraphSelectColor.Text = "Select Pen Color";
            this.button_GraphSelectColor.UseVisualStyleBackColor = true;
            this.button_GraphSelectColor.Click += new System.EventHandler(this.button_GraphSelectColor_Click);
            // 
            // label2
            // 
            this.label2.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.label2.AutoSize = true;
            this.label2.ImageAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.label2.Location = new System.Drawing.Point(230, 33);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(54, 33);
            this.label2.TabIndex = 23;
            this.label2.Text = "V. Scale:";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // numericUpDown_vScale
            // 
            this.numericUpDown_vScale.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.numericUpDown_vScale.DecimalPlaces = 2;
            this.numericUpDown_vScale.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.numericUpDown_vScale.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDown_vScale.Location = new System.Drawing.Point(290, 36);
            this.numericUpDown_vScale.Maximum = new decimal(new int[] {
            80,
            0,
            0,
            0});
            this.numericUpDown_vScale.Minimum = new decimal(new int[] {
            80,
            0,
            0,
            -2147483648});
            this.numericUpDown_vScale.Name = "numericUpDown_vScale";
            this.numericUpDown_vScale.Size = new System.Drawing.Size(61, 26);
            this.numericUpDown_vScale.TabIndex = 24;
            this.numericUpDown_vScale.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericUpDown_vScale.ValueChanged += new System.EventHandler(this.numericUpDown_vScale_ValueChanged);
            // 
            // tbGraphBackColor
            // 
            this.tbGraphBackColor.BackColor = System.Drawing.SystemColors.ControlText;
            this.tbGraphBackColor.Enabled = false;
            this.tbGraphBackColor.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.tbGraphBackColor.Location = new System.Drawing.Point(191, 3);
            this.tbGraphBackColor.Name = "tbGraphBackColor";
            this.tbGraphBackColor.ReadOnly = true;
            this.tbGraphBackColor.Size = new System.Drawing.Size(29, 26);
            this.tbGraphBackColor.TabIndex = 16;
            // 
            // tbGraphLineColor
            // 
            this.tbGraphLineColor.BackColor = System.Drawing.SystemColors.HotTrack;
            this.tbGraphLineColor.Enabled = false;
            this.tbGraphLineColor.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.tbGraphLineColor.Location = new System.Drawing.Point(191, 36);
            this.tbGraphLineColor.Name = "tbGraphLineColor";
            this.tbGraphLineColor.ReadOnly = true;
            this.tbGraphLineColor.Size = new System.Drawing.Size(29, 26);
            this.tbGraphLineColor.TabIndex = 17;
            // 
            // label1
            // 
            this.label1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.label1.AutoSize = true;
            this.label1.ImageAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.label1.Location = new System.Drawing.Point(230, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(54, 33);
            this.label1.TabIndex = 22;
            this.label1.Text = "H. Scale:";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // btnDataAnalysisSelectAll
            // 
            this.btnDataAnalysisSelectAll.Location = new System.Drawing.Point(3, 3);
            this.btnDataAnalysisSelectAll.Name = "btnDataAnalysisSelectAll";
            this.btnDataAnalysisSelectAll.Size = new System.Drawing.Size(73, 27);
            this.btnDataAnalysisSelectAll.TabIndex = 26;
            this.btnDataAnalysisSelectAll.Text = "Select All";
            this.btnDataAnalysisSelectAll.UseVisualStyleBackColor = true;
            this.btnDataAnalysisSelectAll.Click += new System.EventHandler(this.btnDataAnalysisSelectAll_Click);
            // 
            // numericUpDown_hScale
            // 
            this.numericUpDown_hScale.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.numericUpDown_hScale.DecimalPlaces = 1;
            this.numericUpDown_hScale.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.numericUpDown_hScale.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDown_hScale.Location = new System.Drawing.Point(290, 3);
            this.numericUpDown_hScale.Maximum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown_hScale.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDown_hScale.Name = "numericUpDown_hScale";
            this.numericUpDown_hScale.Size = new System.Drawing.Size(61, 26);
            this.numericUpDown_hScale.TabIndex = 21;
            this.numericUpDown_hScale.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericUpDown_hScale.ValueChanged += new System.EventHandler(this.numericUpDown_hScale_ValueChanged);
            // 
            // cb_AltitudeHold
            // 
            this.cb_AltitudeHold.AutoSize = true;
            this.cb_AltitudeHold.Location = new System.Drawing.Point(822, 3);
            this.cb_AltitudeHold.Name = "cb_AltitudeHold";
            this.cb_AltitudeHold.Size = new System.Drawing.Size(112, 17);
            this.cb_AltitudeHold.TabIndex = 28;
            this.cb_AltitudeHold.Text = "ALTITUDE HOLD";
            this.cb_AltitudeHold.UseVisualStyleBackColor = true;
            this.cb_AltitudeHold.CheckedChanged += new System.EventHandler(this.cb_AltitudeHold_CheckedChanged);
            // 
            // button_setallcmd
            // 
            this.button_setallcmd.Location = new System.Drawing.Point(822, 36);
            this.button_setallcmd.Name = "button_setallcmd";
            this.button_setallcmd.Size = new System.Drawing.Size(87, 23);
            this.button_setallcmd.TabIndex = 2;
            this.button_setallcmd.Text = "Set All Cmd";
            this.button_setallcmd.UseVisualStyleBackColor = true;
            this.button_setallcmd.Click += new System.EventHandler(this.button_setallcmd_Click);
            // 
            // tableLayoutPanelDataTx
            // 
            this.tableLayoutPanelDataTx.ColumnCount = 2;
            this.tableLayoutPanelDataTx.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 25.32073F));
            this.tableLayoutPanelDataTx.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 74.67927F));
            this.tableLayoutPanelDataTx.Controls.Add(this.tableLayoutPanel1, 1, 0);
            this.tableLayoutPanelDataTx.Controls.Add(this.tableLayoutPanel10, 0, 0);
            this.tableLayoutPanelDataTx.Location = new System.Drawing.Point(3, 534);
            this.tableLayoutPanelDataTx.Name = "tableLayoutPanelDataTx";
            this.tableLayoutPanelDataTx.RowCount = 1;
            this.tableLayoutPanelDataTx.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanelDataTx.Size = new System.Drawing.Size(1478, 337);
            this.tableLayoutPanelDataTx.TabIndex = 2;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 1.550388F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 98.44962F));
            this.tableLayoutPanel1.Controls.Add(this.tableLayoutPanel7, 1, 0);
            this.tableLayoutPanel1.Location = new System.Drawing.Point(377, 3);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 1;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1098, 331);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // tableLayoutPanel7
            // 
            this.tableLayoutPanel7.ColumnCount = 6;
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 49.23858F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50.76142F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 202F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 217F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 217F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 79F));
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel9, 0, 1);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel8, 0, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel11, 1, 1);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel12, 1, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel13, 2, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel15, 3, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel14, 4, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel16, 4, 1);
            this.tableLayoutPanel7.Controls.Add(this.cb_saveHome, 2, 1);
            this.tableLayoutPanel7.Location = new System.Drawing.Point(20, 3);
            this.tableLayoutPanel7.Name = "tableLayoutPanel7";
            this.tableLayoutPanel7.RowCount = 2;
            this.tableLayoutPanel7.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel7.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel7.Size = new System.Drawing.Size(1075, 325);
            this.tableLayoutPanel7.TabIndex = 1;
            // 
            // tableLayoutPanel9
            // 
            this.tableLayoutPanel9.ColumnCount = 2;
            this.tableLayoutPanel9.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.21053F));
            this.tableLayoutPanel9.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 85.78947F));
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_kp, 1, 1);
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_ki, 1, 2);
            this.tableLayoutPanel9.Controls.Add(this.label8, 0, 2);
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_kd, 1, 3);
            this.tableLayoutPanel9.Controls.Add(this.label9, 0, 3);
            this.tableLayoutPanel9.Controls.Add(this.label10, 1, 0);
            this.tableLayoutPanel9.Controls.Add(this.label7, 0, 1);
            this.tableLayoutPanel9.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel9.Location = new System.Drawing.Point(3, 165);
            this.tableLayoutPanel9.Name = "tableLayoutPanel9";
            this.tableLayoutPanel9.RowCount = 6;
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 17F));
            this.tableLayoutPanel9.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel9.Size = new System.Drawing.Size(171, 157);
            this.tableLayoutPanel9.TabIndex = 0;
            // 
            // tb_pid_rate_pitch_roll_kp
            // 
            this.tb_pid_rate_pitch_roll_kp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_kp.Location = new System.Drawing.Point(27, 20);
            this.tb_pid_rate_pitch_roll_kp.Maximum = 255;
            this.tb_pid_rate_pitch_roll_kp.Name = "tb_pid_rate_pitch_roll_kp";
            this.tb_pid_rate_pitch_roll_kp.Size = new System.Drawing.Size(141, 22);
            this.tb_pid_rate_pitch_roll_kp.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_kp.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_kp.Value = 160;
            this.tb_pid_rate_pitch_roll_kp.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_kp_Scroll);
            // 
            // tb_pid_rate_pitch_roll_ki
            // 
            this.tb_pid_rate_pitch_roll_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_ki.Location = new System.Drawing.Point(27, 48);
            this.tb_pid_rate_pitch_roll_ki.Maximum = 255;
            this.tb_pid_rate_pitch_roll_ki.Name = "tb_pid_rate_pitch_roll_ki";
            this.tb_pid_rate_pitch_roll_ki.Size = new System.Drawing.Size(141, 22);
            this.tb_pid_rate_pitch_roll_ki.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_ki.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_ki.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_ki_Scroll);
            // 
            // label8
            // 
            this.label8.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(4, 52);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(16, 13);
            this.label8.TabIndex = 1;
            this.label8.Text = "Ki";
            this.label8.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_rate_pitch_roll_kd
            // 
            this.tb_pid_rate_pitch_roll_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_kd.Location = new System.Drawing.Point(27, 76);
            this.tb_pid_rate_pitch_roll_kd.Maximum = 255;
            this.tb_pid_rate_pitch_roll_kd.Name = "tb_pid_rate_pitch_roll_kd";
            this.tb_pid_rate_pitch_roll_kd.Size = new System.Drawing.Size(141, 22);
            this.tb_pid_rate_pitch_roll_kd.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_kd.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_kd.Value = 130;
            this.tb_pid_rate_pitch_roll_kd.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_kd_Scroll);
            // 
            // label9
            // 
            this.label9.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(5, 74);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(14, 26);
            this.label9.TabIndex = 1;
            this.label9.Text = "Kd";
            this.label9.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label10
            // 
            this.label10.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(47, 2);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(101, 13);
            this.label10.TabIndex = 1;
            this.label10.Text = "PID Rate Pitch/Roll";
            this.label10.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label7
            // 
            this.label7.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(5, 18);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(14, 26);
            this.label7.TabIndex = 1;
            this.label7.Text = "Kp";
            this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel8
            // 
            this.tableLayoutPanel8.ColumnCount = 2;
            this.tableLayoutPanel8.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 14.21053F));
            this.tableLayoutPanel8.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 85.78947F));
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_kp, 1, 1);
            this.tableLayoutPanel8.Controls.Add(this.label3, 0, 1);
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_ki, 1, 2);
            this.tableLayoutPanel8.Controls.Add(this.label4, 0, 2);
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_kd, 1, 3);
            this.tableLayoutPanel8.Controls.Add(this.label5, 0, 3);
            this.tableLayoutPanel8.Controls.Add(this.label6, 1, 0);
            this.tableLayoutPanel8.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel8.Location = new System.Drawing.Point(3, 3);
            this.tableLayoutPanel8.Name = "tableLayoutPanel8";
            this.tableLayoutPanel8.RowCount = 6;
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel8.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 17F));
            this.tableLayoutPanel8.Size = new System.Drawing.Size(171, 156);
            this.tableLayoutPanel8.TabIndex = 0;
            // 
            // tb_pid_angle_pitch_roll_kp
            // 
            this.tb_pid_angle_pitch_roll_kp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_kp.Location = new System.Drawing.Point(27, 20);
            this.tb_pid_angle_pitch_roll_kp.Maximum = 255;
            this.tb_pid_angle_pitch_roll_kp.Name = "tb_pid_angle_pitch_roll_kp";
            this.tb_pid_angle_pitch_roll_kp.Size = new System.Drawing.Size(141, 22);
            this.tb_pid_angle_pitch_roll_kp.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_kp.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_kp.Value = 40;
            this.tb_pid_angle_pitch_roll_kp.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_kp_Scroll);
            // 
            // label3
            // 
            this.label3.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(5, 18);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(14, 26);
            this.label3.TabIndex = 1;
            this.label3.Text = "Kp";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_pitch_roll_ki
            // 
            this.tb_pid_angle_pitch_roll_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_ki.Location = new System.Drawing.Point(27, 48);
            this.tb_pid_angle_pitch_roll_ki.Maximum = 255;
            this.tb_pid_angle_pitch_roll_ki.Name = "tb_pid_angle_pitch_roll_ki";
            this.tb_pid_angle_pitch_roll_ki.Size = new System.Drawing.Size(141, 22);
            this.tb_pid_angle_pitch_roll_ki.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_ki.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_ki.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_ki_Scroll);
            // 
            // label4
            // 
            this.label4.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(4, 52);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(16, 13);
            this.label4.TabIndex = 1;
            this.label4.Text = "Ki";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_pitch_roll_kd
            // 
            this.tb_pid_angle_pitch_roll_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_kd.Location = new System.Drawing.Point(27, 76);
            this.tb_pid_angle_pitch_roll_kd.Maximum = 255;
            this.tb_pid_angle_pitch_roll_kd.Name = "tb_pid_angle_pitch_roll_kd";
            this.tb_pid_angle_pitch_roll_kd.Size = new System.Drawing.Size(141, 22);
            this.tb_pid_angle_pitch_roll_kd.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_kd.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_kd.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_kd_Scroll);
            // 
            // label5
            // 
            this.label5.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(5, 74);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(14, 26);
            this.label5.TabIndex = 1;
            this.label5.Text = "Kd";
            this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label6
            // 
            this.label6.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(45, 2);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(105, 13);
            this.label6.TabIndex = 1;
            this.label6.Text = "PID Angle Pitch/Roll";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel11
            // 
            this.tableLayoutPanel11.ColumnCount = 2;
            this.tableLayoutPanel11.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.70558F));
            this.tableLayoutPanel11.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 86.29442F));
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_kp, 1, 1);
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_ki, 1, 2);
            this.tableLayoutPanel11.Controls.Add(this.label15, 0, 2);
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_kd, 1, 3);
            this.tableLayoutPanel11.Controls.Add(this.label16, 0, 3);
            this.tableLayoutPanel11.Controls.Add(this.label17, 1, 0);
            this.tableLayoutPanel11.Controls.Add(this.label18, 0, 1);
            this.tableLayoutPanel11.Location = new System.Drawing.Point(180, 165);
            this.tableLayoutPanel11.Name = "tableLayoutPanel11";
            this.tableLayoutPanel11.RowCount = 6;
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 17F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel11.Size = new System.Drawing.Size(176, 157);
            this.tableLayoutPanel11.TabIndex = 0;
            // 
            // tb_pid_rate_yaw_kp
            // 
            this.tb_pid_rate_yaw_kp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_kp.Location = new System.Drawing.Point(27, 20);
            this.tb_pid_rate_yaw_kp.Maximum = 255;
            this.tb_pid_rate_yaw_kp.Name = "tb_pid_rate_yaw_kp";
            this.tb_pid_rate_yaw_kp.Size = new System.Drawing.Size(146, 22);
            this.tb_pid_rate_yaw_kp.TabIndex = 0;
            this.tb_pid_rate_yaw_kp.TickFrequency = 5;
            this.tb_pid_rate_yaw_kp.Value = 140;
            this.tb_pid_rate_yaw_kp.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_kp_Scroll);
            // 
            // tb_pid_rate_yaw_ki
            // 
            this.tb_pid_rate_yaw_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_ki.Location = new System.Drawing.Point(27, 48);
            this.tb_pid_rate_yaw_ki.Maximum = 255;
            this.tb_pid_rate_yaw_ki.Name = "tb_pid_rate_yaw_ki";
            this.tb_pid_rate_yaw_ki.Size = new System.Drawing.Size(146, 22);
            this.tb_pid_rate_yaw_ki.TabIndex = 0;
            this.tb_pid_rate_yaw_ki.TickFrequency = 5;
            this.tb_pid_rate_yaw_ki.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_ki_Scroll);
            // 
            // label15
            // 
            this.label15.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(4, 52);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(16, 13);
            this.label15.TabIndex = 1;
            this.label15.Text = "Ki";
            this.label15.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_rate_yaw_kd
            // 
            this.tb_pid_rate_yaw_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_kd.Location = new System.Drawing.Point(27, 76);
            this.tb_pid_rate_yaw_kd.Maximum = 255;
            this.tb_pid_rate_yaw_kd.Name = "tb_pid_rate_yaw_kd";
            this.tb_pid_rate_yaw_kd.Size = new System.Drawing.Size(146, 22);
            this.tb_pid_rate_yaw_kd.TabIndex = 0;
            this.tb_pid_rate_yaw_kd.TickFrequency = 5;
            this.tb_pid_rate_yaw_kd.Value = 120;
            this.tb_pid_rate_yaw_kd.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_kd_Scroll);
            // 
            // label16
            // 
            this.label16.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(5, 74);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(14, 26);
            this.label16.TabIndex = 1;
            this.label16.Text = "Kd";
            this.label16.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label17
            // 
            this.label17.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label17.AutoSize = true;
            this.label17.Location = new System.Drawing.Point(62, 2);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(75, 13);
            this.label17.TabIndex = 1;
            this.label17.Text = "PID Rate Yaw";
            this.label17.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label18
            // 
            this.label18.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label18.AutoSize = true;
            this.label18.Location = new System.Drawing.Point(5, 18);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(14, 26);
            this.label18.TabIndex = 1;
            this.label18.Text = "Kp";
            this.label18.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel12
            // 
            this.tableLayoutPanel12.ColumnCount = 2;
            this.tableLayoutPanel12.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.95349F));
            this.tableLayoutPanel12.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 86.04651F));
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_kp, 1, 1);
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_ki, 1, 2);
            this.tableLayoutPanel12.Controls.Add(this.label21, 0, 2);
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_kd, 1, 3);
            this.tableLayoutPanel12.Controls.Add(this.label22, 0, 3);
            this.tableLayoutPanel12.Controls.Add(this.label23, 1, 0);
            this.tableLayoutPanel12.Controls.Add(this.label24, 0, 1);
            this.tableLayoutPanel12.Location = new System.Drawing.Point(180, 3);
            this.tableLayoutPanel12.Name = "tableLayoutPanel12";
            this.tableLayoutPanel12.RowCount = 6;
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 17F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel12.Size = new System.Drawing.Size(176, 156);
            this.tableLayoutPanel12.TabIndex = 0;
            // 
            // tb_pid_angle_yaw_kp
            // 
            this.tb_pid_angle_yaw_kp.Location = new System.Drawing.Point(27, 20);
            this.tb_pid_angle_yaw_kp.Maximum = 255;
            this.tb_pid_angle_yaw_kp.Name = "tb_pid_angle_yaw_kp";
            this.tb_pid_angle_yaw_kp.Size = new System.Drawing.Size(146, 22);
            this.tb_pid_angle_yaw_kp.TabIndex = 0;
            this.tb_pid_angle_yaw_kp.TickFrequency = 5;
            this.tb_pid_angle_yaw_kp.Value = 30;
            this.tb_pid_angle_yaw_kp.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_kp_Scroll);
            // 
            // tb_pid_angle_yaw_ki
            // 
            this.tb_pid_angle_yaw_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_yaw_ki.Location = new System.Drawing.Point(27, 48);
            this.tb_pid_angle_yaw_ki.Maximum = 255;
            this.tb_pid_angle_yaw_ki.Name = "tb_pid_angle_yaw_ki";
            this.tb_pid_angle_yaw_ki.Size = new System.Drawing.Size(146, 22);
            this.tb_pid_angle_yaw_ki.TabIndex = 0;
            this.tb_pid_angle_yaw_ki.TickFrequency = 5;
            this.tb_pid_angle_yaw_ki.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_ki_Scroll);
            // 
            // label21
            // 
            this.label21.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label21.AutoSize = true;
            this.label21.Location = new System.Drawing.Point(4, 52);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(16, 13);
            this.label21.TabIndex = 1;
            this.label21.Text = "Ki";
            this.label21.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_yaw_kd
            // 
            this.tb_pid_angle_yaw_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_yaw_kd.Location = new System.Drawing.Point(27, 76);
            this.tb_pid_angle_yaw_kd.Maximum = 255;
            this.tb_pid_angle_yaw_kd.Name = "tb_pid_angle_yaw_kd";
            this.tb_pid_angle_yaw_kd.Size = new System.Drawing.Size(146, 22);
            this.tb_pid_angle_yaw_kd.TabIndex = 0;
            this.tb_pid_angle_yaw_kd.TickFrequency = 5;
            this.tb_pid_angle_yaw_kd.Value = 25;
            this.tb_pid_angle_yaw_kd.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_kd_Scroll);
            // 
            // label22
            // 
            this.label22.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label22.AutoSize = true;
            this.label22.Location = new System.Drawing.Point(5, 74);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(14, 26);
            this.label22.TabIndex = 1;
            this.label22.Text = "Kd";
            this.label22.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label23
            // 
            this.label23.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label23.AutoSize = true;
            this.label23.Location = new System.Drawing.Point(60, 2);
            this.label23.Name = "label23";
            this.label23.Size = new System.Drawing.Size(79, 13);
            this.label23.TabIndex = 1;
            this.label23.Text = "PID Angle Yaw";
            this.label23.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label24
            // 
            this.label24.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label24.AutoSize = true;
            this.label24.Location = new System.Drawing.Point(5, 18);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(14, 26);
            this.label24.TabIndex = 1;
            this.label24.Text = "Kp";
            this.label24.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel13
            // 
            this.tableLayoutPanel13.ColumnCount = 2;
            this.tableLayoutPanel13.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.8756F));
            this.tableLayoutPanel13.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 86.1244F));
            this.tableLayoutPanel13.Controls.Add(this.tb_pid_pos_alt_kp, 1, 1);
            this.tableLayoutPanel13.Controls.Add(this.tb_pid_pos_alt_ki, 1, 2);
            this.tableLayoutPanel13.Controls.Add(this.label11, 0, 2);
            this.tableLayoutPanel13.Controls.Add(this.tb_pid_pos_alt_kd, 1, 3);
            this.tableLayoutPanel13.Controls.Add(this.label12, 0, 3);
            this.tableLayoutPanel13.Controls.Add(this.label13, 1, 0);
            this.tableLayoutPanel13.Controls.Add(this.label14, 0, 1);
            this.tableLayoutPanel13.Location = new System.Drawing.Point(362, 3);
            this.tableLayoutPanel13.Name = "tableLayoutPanel13";
            this.tableLayoutPanel13.RowCount = 5;
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel13.Size = new System.Drawing.Size(196, 156);
            this.tableLayoutPanel13.TabIndex = 3;
            // 
            // tb_pid_pos_alt_kp
            // 
            this.tb_pid_pos_alt_kp.Location = new System.Drawing.Point(30, 23);
            this.tb_pid_pos_alt_kp.Maximum = 255;
            this.tb_pid_pos_alt_kp.Name = "tb_pid_pos_alt_kp";
            this.tb_pid_pos_alt_kp.Size = new System.Drawing.Size(163, 27);
            this.tb_pid_pos_alt_kp.TabIndex = 0;
            this.tb_pid_pos_alt_kp.TickFrequency = 5;
            this.tb_pid_pos_alt_kp.Value = 40;
            this.tb_pid_pos_alt_kp.Scroll += new System.EventHandler(this.tb_pid_pos_alt_kp_Scroll);
            // 
            // tb_pid_pos_alt_ki
            // 
            this.tb_pid_pos_alt_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_pos_alt_ki.Location = new System.Drawing.Point(30, 56);
            this.tb_pid_pos_alt_ki.Maximum = 255;
            this.tb_pid_pos_alt_ki.Name = "tb_pid_pos_alt_ki";
            this.tb_pid_pos_alt_ki.Size = new System.Drawing.Size(163, 27);
            this.tb_pid_pos_alt_ki.TabIndex = 0;
            this.tb_pid_pos_alt_ki.TickFrequency = 5;
            this.tb_pid_pos_alt_ki.Scroll += new System.EventHandler(this.tb_pid_pos_alt_ki_Scroll);
            // 
            // label11
            // 
            this.label11.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(5, 63);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(16, 13);
            this.label11.TabIndex = 1;
            this.label11.Text = "Ki";
            this.label11.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_pos_alt_kd
            // 
            this.tb_pid_pos_alt_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_pos_alt_kd.Location = new System.Drawing.Point(30, 89);
            this.tb_pid_pos_alt_kd.Maximum = 255;
            this.tb_pid_pos_alt_kd.Name = "tb_pid_pos_alt_kd";
            this.tb_pid_pos_alt_kd.Size = new System.Drawing.Size(163, 27);
            this.tb_pid_pos_alt_kd.TabIndex = 0;
            this.tb_pid_pos_alt_kd.TickFrequency = 5;
            this.tb_pid_pos_alt_kd.Scroll += new System.EventHandler(this.tb_pid_pos_alt_kd_Scroll);
            // 
            // label12
            // 
            this.label12.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(3, 96);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(20, 13);
            this.label12.TabIndex = 1;
            this.label12.Text = "Kd";
            this.label12.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label13
            // 
            this.label13.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(81, 3);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(61, 13);
            this.label13.TabIndex = 1;
            this.label13.Text = "PID Pos Alt";
            this.label13.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label14
            // 
            this.label14.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(3, 30);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(20, 13);
            this.label14.TabIndex = 1;
            this.label14.Text = "Kp";
            this.label14.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel15
            // 
            this.tableLayoutPanel15.ColumnCount = 2;
            this.tableLayoutPanel15.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 13.39713F));
            this.tableLayoutPanel15.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 86.60287F));
            this.tableLayoutPanel15.Controls.Add(this.tb_pid_vel_alt_kp, 1, 1);
            this.tableLayoutPanel15.Controls.Add(this.tb_pid_vel_alt_ki, 1, 2);
            this.tableLayoutPanel15.Controls.Add(this.label35, 0, 2);
            this.tableLayoutPanel15.Controls.Add(this.tb_pid_vel_alt_kd, 1, 3);
            this.tableLayoutPanel15.Controls.Add(this.label36, 0, 3);
            this.tableLayoutPanel15.Controls.Add(this.label37, 1, 0);
            this.tableLayoutPanel15.Controls.Add(this.label38, 0, 1);
            this.tableLayoutPanel15.Location = new System.Drawing.Point(564, 3);
            this.tableLayoutPanel15.Name = "tableLayoutPanel15";
            this.tableLayoutPanel15.RowCount = 5;
            this.tableLayoutPanel15.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel15.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel15.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel15.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel15.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel15.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel15.Size = new System.Drawing.Size(209, 156);
            this.tableLayoutPanel15.TabIndex = 0;
            // 
            // tb_pid_vel_alt_kp
            // 
            this.tb_pid_vel_alt_kp.Location = new System.Drawing.Point(31, 23);
            this.tb_pid_vel_alt_kp.Maximum = 255;
            this.tb_pid_vel_alt_kp.Name = "tb_pid_vel_alt_kp";
            this.tb_pid_vel_alt_kp.Size = new System.Drawing.Size(175, 27);
            this.tb_pid_vel_alt_kp.TabIndex = 0;
            this.tb_pid_vel_alt_kp.TickFrequency = 5;
            this.tb_pid_vel_alt_kp.Value = 20;
            this.tb_pid_vel_alt_kp.Scroll += new System.EventHandler(this.tb_pid_vel_alt_kp_Scroll);
            // 
            // tb_pid_vel_alt_ki
            // 
            this.tb_pid_vel_alt_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_vel_alt_ki.Location = new System.Drawing.Point(31, 56);
            this.tb_pid_vel_alt_ki.Maximum = 255;
            this.tb_pid_vel_alt_ki.Name = "tb_pid_vel_alt_ki";
            this.tb_pid_vel_alt_ki.Size = new System.Drawing.Size(175, 27);
            this.tb_pid_vel_alt_ki.TabIndex = 0;
            this.tb_pid_vel_alt_ki.TickFrequency = 5;
            this.tb_pid_vel_alt_ki.Scroll += new System.EventHandler(this.tb_pid_vel_alt_ki_Scroll);
            // 
            // label35
            // 
            this.label35.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label35.AutoSize = true;
            this.label35.Location = new System.Drawing.Point(6, 63);
            this.label35.Name = "label35";
            this.label35.Size = new System.Drawing.Size(16, 13);
            this.label35.TabIndex = 1;
            this.label35.Text = "Ki";
            this.label35.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_vel_alt_kd
            // 
            this.tb_pid_vel_alt_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_vel_alt_kd.Location = new System.Drawing.Point(31, 89);
            this.tb_pid_vel_alt_kd.Maximum = 255;
            this.tb_pid_vel_alt_kd.Name = "tb_pid_vel_alt_kd";
            this.tb_pid_vel_alt_kd.Size = new System.Drawing.Size(175, 27);
            this.tb_pid_vel_alt_kd.TabIndex = 0;
            this.tb_pid_vel_alt_kd.TickFrequency = 5;
            this.tb_pid_vel_alt_kd.Scroll += new System.EventHandler(this.tb_pid_vel_alt_kd_Scroll);
            // 
            // label36
            // 
            this.label36.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label36.AutoSize = true;
            this.label36.Location = new System.Drawing.Point(4, 96);
            this.label36.Name = "label36";
            this.label36.Size = new System.Drawing.Size(20, 13);
            this.label36.TabIndex = 1;
            this.label36.Text = "Kd";
            this.label36.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label37
            // 
            this.label37.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label37.AutoSize = true;
            this.label37.Location = new System.Drawing.Point(89, 3);
            this.label37.Name = "label37";
            this.label37.Size = new System.Drawing.Size(58, 13);
            this.label37.TabIndex = 1;
            this.label37.Text = "PID Vel Alt";
            this.label37.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label38
            // 
            this.label38.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label38.AutoSize = true;
            this.label38.Location = new System.Drawing.Point(4, 30);
            this.label38.Name = "label38";
            this.label38.Size = new System.Drawing.Size(20, 13);
            this.label38.TabIndex = 1;
            this.label38.Text = "Kp";
            this.label38.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel14
            // 
            this.tableLayoutPanel14.ColumnCount = 2;
            this.tableLayoutPanel14.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 12.91866F));
            this.tableLayoutPanel14.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 87.08134F));
            this.tableLayoutPanel14.Controls.Add(this.tb_pid_acc_alt_kp, 1, 1);
            this.tableLayoutPanel14.Controls.Add(this.tb_pid_acc_alt_ki, 1, 2);
            this.tableLayoutPanel14.Controls.Add(this.label29, 0, 2);
            this.tableLayoutPanel14.Controls.Add(this.tb_pid_acc_alt_kd, 1, 3);
            this.tableLayoutPanel14.Controls.Add(this.label30, 0, 3);
            this.tableLayoutPanel14.Controls.Add(this.label31, 1, 0);
            this.tableLayoutPanel14.Controls.Add(this.label32, 0, 1);
            this.tableLayoutPanel14.Location = new System.Drawing.Point(781, 3);
            this.tableLayoutPanel14.Name = "tableLayoutPanel14";
            this.tableLayoutPanel14.RowCount = 5;
            this.tableLayoutPanel14.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel14.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel14.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel14.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel14.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel14.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel14.Size = new System.Drawing.Size(209, 156);
            this.tableLayoutPanel14.TabIndex = 0;
            // 
            // tb_pid_acc_alt_kp
            // 
            this.tb_pid_acc_alt_kp.Location = new System.Drawing.Point(29, 23);
            this.tb_pid_acc_alt_kp.Maximum = 255;
            this.tb_pid_acc_alt_kp.Name = "tb_pid_acc_alt_kp";
            this.tb_pid_acc_alt_kp.Size = new System.Drawing.Size(177, 27);
            this.tb_pid_acc_alt_kp.TabIndex = 0;
            this.tb_pid_acc_alt_kp.TickFrequency = 5;
            this.tb_pid_acc_alt_kp.Value = 20;
            this.tb_pid_acc_alt_kp.Scroll += new System.EventHandler(this.tb_pid_acc_alt_kp_Scroll);
            // 
            // tb_pid_acc_alt_ki
            // 
            this.tb_pid_acc_alt_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_acc_alt_ki.Location = new System.Drawing.Point(29, 56);
            this.tb_pid_acc_alt_ki.Maximum = 255;
            this.tb_pid_acc_alt_ki.Name = "tb_pid_acc_alt_ki";
            this.tb_pid_acc_alt_ki.Size = new System.Drawing.Size(177, 27);
            this.tb_pid_acc_alt_ki.TabIndex = 0;
            this.tb_pid_acc_alt_ki.TickFrequency = 5;
            this.tb_pid_acc_alt_ki.Scroll += new System.EventHandler(this.tb_pid_acc_alt_ki_Scroll);
            // 
            // label29
            // 
            this.label29.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label29.AutoSize = true;
            this.label29.Location = new System.Drawing.Point(5, 63);
            this.label29.Name = "label29";
            this.label29.Size = new System.Drawing.Size(16, 13);
            this.label29.TabIndex = 1;
            this.label29.Text = "Ki";
            this.label29.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_acc_alt_kd
            // 
            this.tb_pid_acc_alt_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_acc_alt_kd.Location = new System.Drawing.Point(29, 89);
            this.tb_pid_acc_alt_kd.Maximum = 255;
            this.tb_pid_acc_alt_kd.Name = "tb_pid_acc_alt_kd";
            this.tb_pid_acc_alt_kd.Size = new System.Drawing.Size(177, 27);
            this.tb_pid_acc_alt_kd.TabIndex = 0;
            this.tb_pid_acc_alt_kd.TickFrequency = 5;
            this.tb_pid_acc_alt_kd.Scroll += new System.EventHandler(this.tb_pid_acc_alt_kd_Scroll);
            // 
            // label30
            // 
            this.label30.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label30.AutoSize = true;
            this.label30.Location = new System.Drawing.Point(3, 96);
            this.label30.Name = "label30";
            this.label30.Size = new System.Drawing.Size(20, 13);
            this.label30.TabIndex = 1;
            this.label30.Text = "Kd";
            this.label30.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label31
            // 
            this.label31.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label31.AutoSize = true;
            this.label31.Location = new System.Drawing.Point(86, 3);
            this.label31.Name = "label31";
            this.label31.Size = new System.Drawing.Size(62, 13);
            this.label31.TabIndex = 1;
            this.label31.Text = "PID Acc Alt";
            this.label31.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label32
            // 
            this.label32.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label32.AutoSize = true;
            this.label32.Location = new System.Drawing.Point(3, 30);
            this.label32.Name = "label32";
            this.label32.Size = new System.Drawing.Size(20, 13);
            this.label32.TabIndex = 1;
            this.label32.Text = "Kp";
            this.label32.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel16
            // 
            this.tableLayoutPanel16.ColumnCount = 2;
            this.tableLayoutPanel16.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 12.91866F));
            this.tableLayoutPanel16.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 87.08134F));
            this.tableLayoutPanel16.Controls.Add(this.tb_pid_acc_posxy_kp, 1, 1);
            this.tableLayoutPanel16.Controls.Add(this.tb_pid_acc_posxy_ki, 1, 2);
            this.tableLayoutPanel16.Controls.Add(this.label19, 0, 2);
            this.tableLayoutPanel16.Controls.Add(this.tb_pid_acc_posxy_kd, 1, 3);
            this.tableLayoutPanel16.Controls.Add(this.label20, 0, 3);
            this.tableLayoutPanel16.Controls.Add(this.label25, 1, 0);
            this.tableLayoutPanel16.Controls.Add(this.label26, 0, 1);
            this.tableLayoutPanel16.Location = new System.Drawing.Point(781, 165);
            this.tableLayoutPanel16.Name = "tableLayoutPanel16";
            this.tableLayoutPanel16.RowCount = 5;
            this.tableLayoutPanel16.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel16.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel16.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel16.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel16.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel16.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel16.Size = new System.Drawing.Size(209, 156);
            this.tableLayoutPanel16.TabIndex = 4;
            // 
            // tb_pid_acc_posxy_kp
            // 
            this.tb_pid_acc_posxy_kp.Location = new System.Drawing.Point(29, 23);
            this.tb_pid_acc_posxy_kp.Maximum = 255;
            this.tb_pid_acc_posxy_kp.Name = "tb_pid_acc_posxy_kp";
            this.tb_pid_acc_posxy_kp.Size = new System.Drawing.Size(177, 27);
            this.tb_pid_acc_posxy_kp.TabIndex = 0;
            this.tb_pid_acc_posxy_kp.TickFrequency = 5;
            this.tb_pid_acc_posxy_kp.Value = 40;
            this.tb_pid_acc_posxy_kp.Scroll += new System.EventHandler(this.tb_pid_acc_posxy_kp_Scroll);
            // 
            // tb_pid_acc_posxy_ki
            // 
            this.tb_pid_acc_posxy_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_acc_posxy_ki.Location = new System.Drawing.Point(29, 56);
            this.tb_pid_acc_posxy_ki.Maximum = 255;
            this.tb_pid_acc_posxy_ki.Name = "tb_pid_acc_posxy_ki";
            this.tb_pid_acc_posxy_ki.Size = new System.Drawing.Size(177, 27);
            this.tb_pid_acc_posxy_ki.TabIndex = 0;
            this.tb_pid_acc_posxy_ki.TickFrequency = 5;
            this.tb_pid_acc_posxy_ki.Scroll += new System.EventHandler(this.tb_pid_acc_posxy_ki_Scroll);
            // 
            // label19
            // 
            this.label19.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label19.AutoSize = true;
            this.label19.Location = new System.Drawing.Point(5, 63);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(16, 13);
            this.label19.TabIndex = 1;
            this.label19.Text = "Ki";
            this.label19.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_acc_posxy_kd
            // 
            this.tb_pid_acc_posxy_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_acc_posxy_kd.Location = new System.Drawing.Point(29, 89);
            this.tb_pid_acc_posxy_kd.Maximum = 255;
            this.tb_pid_acc_posxy_kd.Name = "tb_pid_acc_posxy_kd";
            this.tb_pid_acc_posxy_kd.Size = new System.Drawing.Size(177, 27);
            this.tb_pid_acc_posxy_kd.TabIndex = 0;
            this.tb_pid_acc_posxy_kd.TickFrequency = 5;
            this.tb_pid_acc_posxy_kd.Scroll += new System.EventHandler(this.tb_pid_acc_posxy_kd_Scroll);
            // 
            // label20
            // 
            this.label20.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label20.AutoSize = true;
            this.label20.Location = new System.Drawing.Point(3, 96);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(20, 13);
            this.label20.TabIndex = 1;
            this.label20.Text = "Kd";
            this.label20.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label25
            // 
            this.label25.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label25.AutoSize = true;
            this.label25.Location = new System.Drawing.Point(76, 3);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(82, 13);
            this.label25.TabIndex = 1;
            this.label25.Text = "PID Acc PosXY";
            this.label25.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label26
            // 
            this.label26.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label26.AutoSize = true;
            this.label26.Location = new System.Drawing.Point(3, 30);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(20, 13);
            this.label26.TabIndex = 1;
            this.label26.Text = "Kp";
            this.label26.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // cb_saveHome
            // 
            this.cb_saveHome.AutoSize = true;
            this.cb_saveHome.Location = new System.Drawing.Point(362, 165);
            this.cb_saveHome.Name = "cb_saveHome";
            this.cb_saveHome.Size = new System.Drawing.Size(122, 17);
            this.cb_saveHome.TabIndex = 5;
            this.cb_saveHome.Text = "Save Home Position";
            this.cb_saveHome.UseVisualStyleBackColor = true;
            this.cb_saveHome.CheckedChanged += new System.EventHandler(this.cb_saveHome_CheckedChanged);
            // 
            // tableLayoutPanel10
            // 
            this.tableLayoutPanel10.ColumnCount = 1;
            this.tableLayoutPanel10.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel10.Controls.Add(this.lvDataTx, 0, 0);
            this.tableLayoutPanel10.Controls.Add(this.numericUpDownDataTx, 0, 1);
            this.tableLayoutPanel10.Location = new System.Drawing.Point(3, 3);
            this.tableLayoutPanel10.Name = "tableLayoutPanel10";
            this.tableLayoutPanel10.RowCount = 3;
            this.tableLayoutPanel10.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 86.95652F));
            this.tableLayoutPanel10.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.04348F));
            this.tableLayoutPanel10.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 8F));
            this.tableLayoutPanel10.Size = new System.Drawing.Size(366, 331);
            this.tableLayoutPanel10.TabIndex = 3;
            // 
            // lvDataTx
            // 
            this.lvDataTx.CheckBoxes = true;
            this.lvDataTx.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnHeader1,
            this.columnHeader2});
            this.lvDataTx.Font = new System.Drawing.Font("Courier New", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.lvDataTx.FullRowSelect = true;
            this.lvDataTx.Location = new System.Drawing.Point(3, 3);
            this.lvDataTx.MultiSelect = false;
            this.lvDataTx.Name = "lvDataTx";
            this.lvDataTx.Size = new System.Drawing.Size(360, 274);
            this.lvDataTx.TabIndex = 2;
            this.lvDataTx.UseCompatibleStateImageBehavior = false;
            this.lvDataTx.View = System.Windows.Forms.View.Details;
            // 
            // columnHeader1
            // 
            this.columnHeader1.Text = "Variable Name";
            this.columnHeader1.Width = 224;
            // 
            // columnHeader2
            // 
            this.columnHeader2.Text = "Value";
            this.columnHeader2.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.columnHeader2.Width = 100;
            // 
            // numericUpDownDataTx
            // 
            this.numericUpDownDataTx.DecimalPlaces = 2;
            this.numericUpDownDataTx.Font = new System.Drawing.Font("Microsoft Sans Serif", 20F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.numericUpDownDataTx.Location = new System.Drawing.Point(3, 283);
            this.numericUpDownDataTx.Maximum = new decimal(new int[] {
            32567,
            0,
            0,
            0});
            this.numericUpDownDataTx.Minimum = new decimal(new int[] {
            32567,
            0,
            0,
            -2147483648});
            this.numericUpDownDataTx.Name = "numericUpDownDataTx";
            this.numericUpDownDataTx.Size = new System.Drawing.Size(93, 38);
            this.numericUpDownDataTx.TabIndex = 0;
            this.numericUpDownDataTx.ValueChanged += new System.EventHandler(this.numericUpDownDataTx_ValueChanged);
            // 
            // tableLayoutPanel17
            // 
            this.tableLayoutPanel17.ColumnCount = 3;
            this.tableLayoutPanel17.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 23.67021F));
            this.tableLayoutPanel17.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 76.32979F));
            this.tableLayoutPanel17.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 1101F));
            this.tableLayoutPanel17.Controls.Add(this.label_packet_size, 1, 1);
            this.tableLayoutPanel17.Controls.Add(this.b_open_file, 0, 0);
            this.tableLayoutPanel17.Controls.Add(this.label_record, 1, 0);
            this.tableLayoutPanel17.Controls.Add(this.b_play, 0, 1);
            this.tableLayoutPanel17.Controls.Add(this.trackBar_record, 2, 0);
            this.tableLayoutPanel17.Controls.Add(this.tableLayoutPanel18, 2, 1);
            this.tableLayoutPanel17.Location = new System.Drawing.Point(3, 877);
            this.tableLayoutPanel17.Name = "tableLayoutPanel17";
            this.tableLayoutPanel17.RowCount = 2;
            this.tableLayoutPanel17.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel17.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 42F));
            this.tableLayoutPanel17.Size = new System.Drawing.Size(1478, 68);
            this.tableLayoutPanel17.TabIndex = 3;
            // 
            // label_packet_size
            // 
            this.label_packet_size.AutoSize = true;
            this.label_packet_size.Font = new System.Drawing.Font("Calibri", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label_packet_size.Location = new System.Drawing.Point(92, 26);
            this.label_packet_size.Name = "label_packet_size";
            this.label_packet_size.Padding = new System.Windows.Forms.Padding(0, 5, 0, 0);
            this.label_packet_size.Size = new System.Drawing.Size(79, 20);
            this.label_packet_size.TabIndex = 10;
            this.label_packet_size.Text = "Packet Size: #";
            this.label_packet_size.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // b_open_file
            // 
            this.b_open_file.Location = new System.Drawing.Point(3, 3);
            this.b_open_file.Name = "b_open_file";
            this.b_open_file.Size = new System.Drawing.Size(82, 20);
            this.b_open_file.TabIndex = 6;
            this.b_open_file.Text = "Open Recod File";
            this.b_open_file.UseVisualStyleBackColor = true;
            this.b_open_file.Click += new System.EventHandler(this.button1_Click);
            // 
            // label_record
            // 
            this.label_record.AutoSize = true;
            this.label_record.Font = new System.Drawing.Font("Calibri", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.label_record.Location = new System.Drawing.Point(92, 0);
            this.label_record.Name = "label_record";
            this.label_record.Padding = new System.Windows.Forms.Padding(0, 5, 0, 0);
            this.label_record.Size = new System.Drawing.Size(68, 20);
            this.label_record.TabIndex = 7;
            this.label_record.Text = "Record File";
            this.label_record.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // b_play
            // 
            this.b_play.Location = new System.Drawing.Point(3, 29);
            this.b_play.Name = "b_play";
            this.b_play.Size = new System.Drawing.Size(82, 23);
            this.b_play.TabIndex = 8;
            this.b_play.Text = "Play";
            this.b_play.UseVisualStyleBackColor = true;
            this.b_play.Click += new System.EventHandler(this.b_play_Click);
            // 
            // trackBar_record
            // 
            this.trackBar_record.Location = new System.Drawing.Point(379, 3);
            this.trackBar_record.Maximum = 100;
            this.trackBar_record.Name = "trackBar_record";
            this.trackBar_record.Size = new System.Drawing.Size(829, 20);
            this.trackBar_record.TabIndex = 9;
            this.trackBar_record.Scroll += new System.EventHandler(this.trackBar_record_Scroll);
            // 
            // tableLayoutPanel18
            // 
            this.tableLayoutPanel18.ColumnCount = 2;
            this.tableLayoutPanel18.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel18.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel18.Controls.Add(this.cb_record_save, 0, 0);
            this.tableLayoutPanel18.Controls.Add(this.cb_record_display, 1, 0);
            this.tableLayoutPanel18.Location = new System.Drawing.Point(379, 29);
            this.tableLayoutPanel18.Name = "tableLayoutPanel18";
            this.tableLayoutPanel18.RowCount = 1;
            this.tableLayoutPanel18.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel18.Size = new System.Drawing.Size(200, 36);
            this.tableLayoutPanel18.TabIndex = 11;
            // 
            // cb_record_save
            // 
            this.cb_record_save.AutoSize = true;
            this.cb_record_save.Checked = true;
            this.cb_record_save.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cb_record_save.Location = new System.Drawing.Point(3, 3);
            this.cb_record_save.Name = "cb_record_save";
            this.cb_record_save.Size = new System.Drawing.Size(51, 17);
            this.cb_record_save.TabIndex = 0;
            this.cb_record_save.Text = "Save";
            this.cb_record_save.UseVisualStyleBackColor = true;
            // 
            // cb_record_display
            // 
            this.cb_record_display.AutoSize = true;
            this.cb_record_display.Checked = true;
            this.cb_record_display.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cb_record_display.Location = new System.Drawing.Point(103, 3);
            this.cb_record_display.Name = "cb_record_display";
            this.cb_record_display.Size = new System.Drawing.Size(60, 17);
            this.cb_record_display.TabIndex = 1;
            this.cb_record_display.Text = "Display";
            this.cb_record_display.UseVisualStyleBackColor = true;
            // 
            // bwUdpTransmit
            // 
            this.bwUdpTransmit.DoWork += new System.ComponentModel.DoWorkEventHandler(this.bwUdpTransmit_DoWork);
            // 
            // timer_record
            // 
            this.timer_record.Interval = 10;
            this.timer_record.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // ofd_record
            // 
            this.ofd_record.FileName = "openFileDialog1";
            // 
            // Ground_Station
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1484, 995);
            this.Controls.Add(this.pnlContentPanel);
            this.Controls.Add(this.pnlBottomPanel);
            this.Controls.Add(this.pnlTopPanel);
            this.Name = "Ground_Station";
            this.Text = "Ground Station v0.1";
            this.Load += new System.EventHandler(this.Ground_Station_Load);
            this.pnlTopPanel.ResumeLayout(false);
            this.pnlBottomPanel.ResumeLayout(false);
            this.pnlBottomPanel.PerformLayout();
            this.ssMain.ResumeLayout(false);
            this.ssMain.PerformLayout();
            this.pnlContentPanel.ResumeLayout(false);
            this.tableLayoutContent.ResumeLayout(false);
            this.tableLayoutPanelDataAnalysis.ResumeLayout(false);
            this.tableLayoutPanel2.ResumeLayout(false);
            this.tableLayoutPanel4.ResumeLayout(false);
            this.tableLayoutPanel4.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_hOffset)).EndInit();
            this.tableLayoutPanel6.ResumeLayout(false);
            this.tableLayoutPanel6.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_vOffset)).EndInit();
            this.tableLayoutPanel3.ResumeLayout(false);
            this.tableLayoutPanel3.PerformLayout();
            this.tableLayoutPanel5.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_vScale)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_hScale)).EndInit();
            this.tableLayoutPanelDataTx.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel7.ResumeLayout(false);
            this.tableLayoutPanel7.PerformLayout();
            this.tableLayoutPanel9.ResumeLayout(false);
            this.tableLayoutPanel9.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kd)).EndInit();
            this.tableLayoutPanel8.ResumeLayout(false);
            this.tableLayoutPanel8.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kd)).EndInit();
            this.tableLayoutPanel11.ResumeLayout(false);
            this.tableLayoutPanel11.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kd)).EndInit();
            this.tableLayoutPanel12.ResumeLayout(false);
            this.tableLayoutPanel12.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kd)).EndInit();
            this.tableLayoutPanel13.ResumeLayout(false);
            this.tableLayoutPanel13.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_pos_alt_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_pos_alt_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_pos_alt_kd)).EndInit();
            this.tableLayoutPanel15.ResumeLayout(false);
            this.tableLayoutPanel15.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_vel_alt_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_vel_alt_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_vel_alt_kd)).EndInit();
            this.tableLayoutPanel14.ResumeLayout(false);
            this.tableLayoutPanel14.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_alt_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_alt_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_alt_kd)).EndInit();
            this.tableLayoutPanel16.ResumeLayout(false);
            this.tableLayoutPanel16.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_posxy_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_posxy_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_acc_posxy_kd)).EndInit();
            this.tableLayoutPanel10.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownDataTx)).EndInit();
            this.tableLayoutPanel17.ResumeLayout(false);
            this.tableLayoutPanel17.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_record)).EndInit();
            this.tableLayoutPanel18.ResumeLayout(false);
            this.tableLayoutPanel18.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Panel pnlTopPanel;
        private System.Windows.Forms.Panel pnlBottomPanel;
        private System.Windows.Forms.Panel pnlHeartBeat;
        private System.Windows.Forms.StatusStrip ssMain;
        private System.Windows.Forms.ToolStripStatusLabel ssMainLabel1;
        private System.Windows.Forms.ToolStripStatusLabel ssMainLabel2;
        private System.Windows.Forms.ToolStripStatusLabel ssMainLabel3;
        private System.Windows.Forms.ToolStripStatusLabel ssMainLabel4;
        private System.Windows.Forms.ToolStripProgressBar prbMainProgress;
        private System.Windows.Forms.Timer timerDisplayRefresh;
        private System.Windows.Forms.ColorDialog colorDialog_Graph;
        private System.Windows.Forms.TableLayoutPanel tableLayoutContent;
        private System.Windows.Forms.TrackBar trackBar_hOffset;
        private System.Windows.Forms.TextBox textBox_trackBarGraphHor;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel4;
        private System.Windows.Forms.Button button_GraphSetDefault;
        private System.Windows.Forms.Button button_InsertGraph;
        private System.Windows.Forms.Button button_RemoveGraph;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel5;
        private System.Windows.Forms.NumericUpDown numericUpDown_vScale;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.NumericUpDown numericUpDown_hScale;
        private System.Windows.Forms.TextBox tbGraphLineColor;
        private System.Windows.Forms.Button button_GraphSelectColor;
        private System.Windows.Forms.TextBox tbGraphBackColor;
        private System.Windows.Forms.Button button_GraphSelectBackColor;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel3;
        private System.Windows.Forms.Panel pnlDataAnalysis;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel2;
        private System.Windows.Forms.ColumnHeader columnValue;
        private System.Windows.Forms.ColumnHeader columnVariableName;
        private System.Windows.Forms.ListView lvDataAnalysis;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanelDataAnalysis;
        private System.Windows.Forms.Panel pnlContentPanel;
        private System.Windows.Forms.TrackBar trackBar_vOffset;
        private System.Windows.Forms.TextBox textBox_trackBarGraphVer;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel6;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanelDataTx;
        private System.Windows.Forms.Button btnDataAnalysisDeselectAll;
        private System.Windows.Forms.Button btnDataAnalysisSelectAll;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.NumericUpDown numericUpDownDataTx;
        private System.ComponentModel.BackgroundWorker bwUdpTransmit;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel7;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel8;
        private System.Windows.Forms.TrackBar tb_pid_angle_pitch_roll_kp;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TrackBar tb_pid_angle_pitch_roll_ki;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TrackBar tb_pid_angle_pitch_roll_kd;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel9;
        private System.Windows.Forms.TrackBar tb_pid_rate_pitch_roll_kp;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TrackBar tb_pid_rate_pitch_roll_kd;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel10;
        private System.Windows.Forms.ColumnHeader columnHeader2;
        private System.Windows.Forms.ColumnHeader columnHeader1;
        private System.Windows.Forms.ListView lvDataTx;
        private System.Windows.Forms.TrackBar tb_pid_rate_pitch_roll_ki;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel11;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_kp;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_ki;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_kd;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel12;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_kp;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_ki;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_kd;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.TrackBar tb_pid_acc_alt_kp;
        private System.Windows.Forms.TrackBar tb_pid_acc_alt_ki;
        private System.Windows.Forms.Label label29;
        private System.Windows.Forms.TrackBar tb_pid_acc_alt_kd;
        private System.Windows.Forms.Label label30;
        private System.Windows.Forms.Label label31;
        private System.Windows.Forms.Label label32;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel15;
        private System.Windows.Forms.TrackBar tb_pid_vel_alt_kp;
        private System.Windows.Forms.TrackBar tb_pid_vel_alt_ki;
        private System.Windows.Forms.Label label35;
        private System.Windows.Forms.TrackBar tb_pid_vel_alt_kd;
        private System.Windows.Forms.Label label36;
        private System.Windows.Forms.Label label37;
        private System.Windows.Forms.Label label38;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel14;
        private System.Windows.Forms.CheckBox cb_AltitudeHold;
        private System.Windows.Forms.Button button_setallcmd;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel13;
        private System.Windows.Forms.TrackBar tb_pid_pos_alt_kp;
        private System.Windows.Forms.TrackBar tb_pid_pos_alt_ki;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TrackBar tb_pid_pos_alt_kd;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel16;
        private System.Windows.Forms.TrackBar tb_pid_acc_posxy_kp;
        private System.Windows.Forms.TrackBar tb_pid_acc_posxy_ki;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.TrackBar tb_pid_acc_posxy_kd;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.CheckBox cb_saveHome;
        private System.Windows.Forms.Timer timer_record;
        private System.Windows.Forms.Button b_open_file;
        private System.Windows.Forms.OpenFileDialog ofd_record;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel17;
        private System.Windows.Forms.Label label_record;
        private System.Windows.Forms.Button b_play;
        private System.Windows.Forms.TrackBar trackBar_record;
        private System.Windows.Forms.Label label_packet_size;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel18;
        private System.Windows.Forms.CheckBox cb_record_save;
        private System.Windows.Forms.CheckBox cb_record_display;
    }
}

