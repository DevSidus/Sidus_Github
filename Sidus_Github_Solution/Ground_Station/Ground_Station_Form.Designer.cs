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
            this.bwUdpSniffer = new System.ComponentModel.BackgroundWorker();
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
            this.numericUpDown_hScale = new System.Windows.Forms.NumericUpDown();
            this.numericUpDown_vScale = new System.Windows.Forms.NumericUpDown();
            this.tbGraphBackColor = new System.Windows.Forms.TextBox();
            this.tbGraphLineColor = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.btnDataAnalysisSelectAll = new System.Windows.Forms.Button();
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
            this.tb_pid_rate_pitch_roll_f1 = new System.Windows.Forms.TrackBar();
            this.tb_pid_rate_pitch_roll_f2 = new System.Windows.Forms.TrackBar();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.tableLayoutPanel8 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_angle_pitch_roll_kp = new System.Windows.Forms.TrackBar();
            this.label3 = new System.Windows.Forms.Label();
            this.tb_pid_angle_pitch_roll_ki = new System.Windows.Forms.TrackBar();
            this.label4 = new System.Windows.Forms.Label();
            this.tb_pid_angle_pitch_roll_kd = new System.Windows.Forms.TrackBar();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.tb_pid_angle_pitch_roll_f1 = new System.Windows.Forms.TrackBar();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.tb_pid_angle_pitch_roll_f2 = new System.Windows.Forms.TrackBar();
            this.tableLayoutPanel11 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_rate_yaw_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_rate_yaw_ki = new System.Windows.Forms.TrackBar();
            this.label15 = new System.Windows.Forms.Label();
            this.tb_pid_rate_yaw_kd = new System.Windows.Forms.TrackBar();
            this.label16 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.label18 = new System.Windows.Forms.Label();
            this.tb_pid_rate_yaw_f1 = new System.Windows.Forms.TrackBar();
            this.tb_pid_rate_yaw_f2 = new System.Windows.Forms.TrackBar();
            this.label19 = new System.Windows.Forms.Label();
            this.label20 = new System.Windows.Forms.Label();
            this.tableLayoutPanel12 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_angle_yaw_kp = new System.Windows.Forms.TrackBar();
            this.tb_pid_angle_yaw_ki = new System.Windows.Forms.TrackBar();
            this.label21 = new System.Windows.Forms.Label();
            this.tb_pid_angle_yaw_kd = new System.Windows.Forms.TrackBar();
            this.label22 = new System.Windows.Forms.Label();
            this.label23 = new System.Windows.Forms.Label();
            this.label24 = new System.Windows.Forms.Label();
            this.tb_pid_angle_yaw_f1 = new System.Windows.Forms.TrackBar();
            this.tb_pid_angle_yaw_f2 = new System.Windows.Forms.TrackBar();
            this.label25 = new System.Windows.Forms.Label();
            this.label26 = new System.Windows.Forms.Label();
            this.tableLayoutPanel13 = new System.Windows.Forms.TableLayoutPanel();
            this.tb_pid_angle_pitch_roll_out_filter = new System.Windows.Forms.TrackBar();
            this.label27 = new System.Windows.Forms.Label();
            this.label28 = new System.Windows.Forms.Label();
            this.tb_pid_angle_yaw_out_filter = new System.Windows.Forms.TrackBar();
            this.tableLayoutPanel10 = new System.Windows.Forms.TableLayoutPanel();
            this.lvDataTx = new System.Windows.Forms.ListView();
            this.columnHeader1 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader2 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.numericUpDownDataTx = new System.Windows.Forms.NumericUpDown();
            this.bwUdpTransmit = new System.ComponentModel.BackgroundWorker();
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
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
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_hScale)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_vScale)).BeginInit();
            this.tableLayoutPanelDataTx.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.tableLayoutPanel7.SuspendLayout();
            this.tableLayoutPanel9.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kd)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_f1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_f2)).BeginInit();
            this.tableLayoutPanel8.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kd)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_f1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_f2)).BeginInit();
            this.tableLayoutPanel11.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kd)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_f1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_f2)).BeginInit();
            this.tableLayoutPanel12.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kp)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_ki)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kd)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_f1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_f2)).BeginInit();
            this.tableLayoutPanel13.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_out_filter)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_out_filter)).BeginInit();
            this.tableLayoutPanel10.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownDataTx)).BeginInit();
            this.SuspendLayout();
            // 
            // bwUdpSniffer
            // 
            this.bwUdpSniffer.DoWork += new System.ComponentModel.DoWorkEventHandler(this.bwUdpSniffer_DoWork);
            // 
            // pnlTopPanel
            // 
            this.pnlTopPanel.Controls.Add(this.pnlHeartBeat);
            this.pnlTopPanel.Dock = System.Windows.Forms.DockStyle.Top;
            this.pnlTopPanel.Location = new System.Drawing.Point(0, 0);
            this.pnlTopPanel.Name = "pnlTopPanel";
            this.pnlTopPanel.Size = new System.Drawing.Size(1284, 10);
            this.pnlTopPanel.TabIndex = 0;
            // 
            // pnlHeartBeat
            // 
            this.pnlHeartBeat.BackColor = System.Drawing.Color.OrangeRed;
            this.pnlHeartBeat.Dock = System.Windows.Forms.DockStyle.Top;
            this.pnlHeartBeat.Location = new System.Drawing.Point(0, 0);
            this.pnlHeartBeat.Name = "pnlHeartBeat";
            this.pnlHeartBeat.Size = new System.Drawing.Size(1284, 10);
            this.pnlHeartBeat.TabIndex = 0;
            // 
            // pnlBottomPanel
            // 
            this.pnlBottomPanel.Controls.Add(this.ssMain);
            this.pnlBottomPanel.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.pnlBottomPanel.Location = new System.Drawing.Point(0, 879);
            this.pnlBottomPanel.Name = "pnlBottomPanel";
            this.pnlBottomPanel.Size = new System.Drawing.Size(1284, 32);
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
            this.ssMain.Size = new System.Drawing.Size(1284, 22);
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
            this.timerDisplayRefresh.Interval = 20;
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
            this.pnlContentPanel.Size = new System.Drawing.Size(1284, 874);
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
            this.tableLayoutContent.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutContent.Name = "tableLayoutContent";
            this.tableLayoutContent.RowCount = 2;
            this.tableLayoutContent.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutContent.RowStyles.Add(new System.Windows.Forms.RowStyle());
            this.tableLayoutContent.Size = new System.Drawing.Size(1347, 874);
            this.tableLayoutContent.TabIndex = 0;
            // 
            // tableLayoutPanelDataAnalysis
            // 
            this.tableLayoutPanelDataAnalysis.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanelDataAnalysis.ColumnCount = 2;
            this.tableLayoutPanelDataAnalysis.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 373F));
            this.tableLayoutPanelDataAnalysis.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 968F));
            this.tableLayoutPanelDataAnalysis.Controls.Add(this.lvDataAnalysis, 0, 0);
            this.tableLayoutPanelDataAnalysis.Controls.Add(this.tableLayoutPanel2, 1, 0);
            this.tableLayoutPanelDataAnalysis.Location = new System.Drawing.Point(3, 3);
            this.tableLayoutPanelDataAnalysis.Name = "tableLayoutPanelDataAnalysis";
            this.tableLayoutPanelDataAnalysis.RowCount = 1;
            this.tableLayoutPanelDataAnalysis.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanelDataAnalysis.Size = new System.Drawing.Size(1341, 525);
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
            this.tableLayoutPanel2.Size = new System.Drawing.Size(962, 519);
            this.tableLayoutPanel2.TabIndex = 1;
            // 
            // pnlDataAnalysis
            // 
            this.pnlDataAnalysis.Location = new System.Drawing.Point(3, 3);
            this.pnlDataAnalysis.Name = "pnlDataAnalysis";
            this.pnlDataAnalysis.Size = new System.Drawing.Size(848, 406);
            this.pnlDataAnalysis.TabIndex = 6;
            // 
            // tableLayoutPanel4
            // 
            this.tableLayoutPanel4.ColumnCount = 2;
            this.tableLayoutPanel4.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 95.51887F));
            this.tableLayoutPanel4.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 4.481132F));
            this.tableLayoutPanel4.Controls.Add(this.textBox_trackBarGraphHor, 0, 0);
            this.tableLayoutPanel4.Controls.Add(this.trackBar_hOffset, 0, 0);
            this.tableLayoutPanel4.Location = new System.Drawing.Point(3, 415);
            this.tableLayoutPanel4.Name = "tableLayoutPanel4";
            this.tableLayoutPanel4.RowCount = 1;
            this.tableLayoutPanel4.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel4.Size = new System.Drawing.Size(848, 28);
            this.tableLayoutPanel4.TabIndex = 8;
            // 
            // textBox_trackBarGraphHor
            // 
            this.textBox_trackBarGraphHor.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.textBox_trackBarGraphHor.Location = new System.Drawing.Point(812, 3);
            this.textBox_trackBarGraphHor.Name = "textBox_trackBarGraphHor";
            this.textBox_trackBarGraphHor.Size = new System.Drawing.Size(33, 20);
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
            this.trackBar_hOffset.Size = new System.Drawing.Size(789, 22);
            this.trackBar_hOffset.TabIndex = 6;
            this.trackBar_hOffset.Scroll += new System.EventHandler(this.trackBar_hOffset_Scroll);
            // 
            // tableLayoutPanel6
            // 
            this.tableLayoutPanel6.ColumnCount = 1;
            this.tableLayoutPanel6.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel6.Controls.Add(this.textBox_trackBarGraphVer, 0, 1);
            this.tableLayoutPanel6.Controls.Add(this.trackBar_vOffset, 0, 0);
            this.tableLayoutPanel6.Location = new System.Drawing.Point(857, 3);
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
            this.tableLayoutPanel3.ColumnCount = 6;
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 79F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 109F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 39F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 60F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 11.4082F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 88.5918F));
            this.tableLayoutPanel3.Controls.Add(this.btnDataAnalysisDeselectAll, 0, 1);
            this.tableLayoutPanel3.Controls.Add(this.tableLayoutPanel5, 5, 0);
            this.tableLayoutPanel3.Controls.Add(this.button_GraphSelectBackColor, 1, 0);
            this.tableLayoutPanel3.Controls.Add(this.button_GraphSelectColor, 1, 1);
            this.tableLayoutPanel3.Controls.Add(this.label2, 3, 1);
            this.tableLayoutPanel3.Controls.Add(this.numericUpDown_hScale, 4, 0);
            this.tableLayoutPanel3.Controls.Add(this.numericUpDown_vScale, 4, 1);
            this.tableLayoutPanel3.Controls.Add(this.tbGraphBackColor, 2, 0);
            this.tableLayoutPanel3.Controls.Add(this.tbGraphLineColor, 2, 1);
            this.tableLayoutPanel3.Controls.Add(this.label1, 3, 0);
            this.tableLayoutPanel3.Controls.Add(this.btnDataAnalysisSelectAll, 0, 0);
            this.tableLayoutPanel3.Location = new System.Drawing.Point(3, 449);
            this.tableLayoutPanel3.Name = "tableLayoutPanel3";
            this.tableLayoutPanel3.RowCount = 2;
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel3.Size = new System.Drawing.Size(848, 66);
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
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 30.76923F));
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 69.23077F));
            this.tableLayoutPanel5.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 290F));
            this.tableLayoutPanel5.Controls.Add(this.button_GraphSetDefault, 0, 0);
            this.tableLayoutPanel5.Controls.Add(this.button_RemoveGraph, 2, 0);
            this.tableLayoutPanel5.Controls.Add(this.button_InsertGraph, 1, 0);
            this.tableLayoutPanel5.Location = new System.Drawing.Point(354, 3);
            this.tableLayoutPanel5.Name = "tableLayoutPanel5";
            this.tableLayoutPanel5.RowCount = 1;
            this.tableLayoutPanel5.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel5.Size = new System.Drawing.Size(491, 27);
            this.tableLayoutPanel5.TabIndex = 25;
            // 
            // button_GraphSetDefault
            // 
            this.button_GraphSetDefault.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.button_GraphSetDefault.Location = new System.Drawing.Point(3, 3);
            this.button_GraphSetDefault.Name = "button_GraphSetDefault";
            this.button_GraphSetDefault.Size = new System.Drawing.Size(55, 21);
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
            this.button_RemoveGraph.Location = new System.Drawing.Point(203, 3);
            this.button_RemoveGraph.Name = "button_RemoveGraph";
            this.button_RemoveGraph.Size = new System.Drawing.Size(285, 21);
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
            this.button_InsertGraph.Location = new System.Drawing.Point(64, 3);
            this.button_InsertGraph.Name = "button_InsertGraph";
            this.button_InsertGraph.Size = new System.Drawing.Size(133, 21);
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
            this.numericUpDown_hScale.Size = new System.Drawing.Size(55, 26);
            this.numericUpDown_hScale.TabIndex = 21;
            this.numericUpDown_hScale.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericUpDown_hScale.ValueChanged += new System.EventHandler(this.numericUpDown_hScale_ValueChanged);
            // 
            // numericUpDown_vScale
            // 
            this.numericUpDown_vScale.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left)));
            this.numericUpDown_vScale.DecimalPlaces = 1;
            this.numericUpDown_vScale.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(162)));
            this.numericUpDown_vScale.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericUpDown_vScale.Location = new System.Drawing.Point(290, 36);
            this.numericUpDown_vScale.Maximum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.numericUpDown_vScale.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            -2147483648});
            this.numericUpDown_vScale.Name = "numericUpDown_vScale";
            this.numericUpDown_vScale.Size = new System.Drawing.Size(55, 26);
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
            // tableLayoutPanelDataTx
            // 
            this.tableLayoutPanelDataTx.ColumnCount = 2;
            this.tableLayoutPanelDataTx.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 29.11788F));
            this.tableLayoutPanelDataTx.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 70.88213F));
            this.tableLayoutPanelDataTx.Controls.Add(this.tableLayoutPanel1, 1, 0);
            this.tableLayoutPanelDataTx.Controls.Add(this.tableLayoutPanel10, 0, 0);
            this.tableLayoutPanelDataTx.Location = new System.Drawing.Point(3, 534);
            this.tableLayoutPanelDataTx.Name = "tableLayoutPanelDataTx";
            this.tableLayoutPanelDataTx.RowCount = 1;
            this.tableLayoutPanelDataTx.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanelDataTx.Size = new System.Drawing.Size(1281, 337);
            this.tableLayoutPanelDataTx.TabIndex = 2;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 2;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 1.550388F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 98.44962F));
            this.tableLayoutPanel1.Controls.Add(this.tableLayoutPanel7, 1, 0);
            this.tableLayoutPanel1.Location = new System.Drawing.Point(375, 3);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 1;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(903, 331);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // tableLayoutPanel7
            // 
            this.tableLayoutPanel7.ColumnCount = 3;
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 49.0939F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50.9061F));
            this.tableLayoutPanel7.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 289F));
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel9, 0, 1);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel8, 0, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel11, 1, 1);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel12, 1, 0);
            this.tableLayoutPanel7.Controls.Add(this.tableLayoutPanel13, 2, 0);
            this.tableLayoutPanel7.Location = new System.Drawing.Point(17, 3);
            this.tableLayoutPanel7.Name = "tableLayoutPanel7";
            this.tableLayoutPanel7.RowCount = 2;
            this.tableLayoutPanel7.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel7.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel7.Size = new System.Drawing.Size(883, 325);
            this.tableLayoutPanel7.TabIndex = 1;
            // 
            // tableLayoutPanel9
            // 
            this.tableLayoutPanel9.ColumnCount = 2;
            this.tableLayoutPanel9.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10.25641F));
            this.tableLayoutPanel9.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 89.74359F));
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_kp, 1, 1);
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_ki, 1, 2);
            this.tableLayoutPanel9.Controls.Add(this.label8, 0, 2);
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_kd, 1, 3);
            this.tableLayoutPanel9.Controls.Add(this.label9, 0, 3);
            this.tableLayoutPanel9.Controls.Add(this.label10, 1, 0);
            this.tableLayoutPanel9.Controls.Add(this.label7, 0, 1);
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_f1, 1, 4);
            this.tableLayoutPanel9.Controls.Add(this.tb_pid_rate_pitch_roll_f2, 1, 5);
            this.tableLayoutPanel9.Controls.Add(this.label11, 0, 4);
            this.tableLayoutPanel9.Controls.Add(this.label12, 0, 5);
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
            this.tableLayoutPanel9.Size = new System.Drawing.Size(285, 157);
            this.tableLayoutPanel9.TabIndex = 0;
            // 
            // tb_pid_rate_pitch_roll_kp
            // 
            this.tb_pid_rate_pitch_roll_kp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_kp.Location = new System.Drawing.Point(32, 20);
            this.tb_pid_rate_pitch_roll_kp.Maximum = 255;
            this.tb_pid_rate_pitch_roll_kp.Name = "tb_pid_rate_pitch_roll_kp";
            this.tb_pid_rate_pitch_roll_kp.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_rate_pitch_roll_kp.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_kp.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_kp.Value = 90;
            this.tb_pid_rate_pitch_roll_kp.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_kp_Scroll);
            // 
            // tb_pid_rate_pitch_roll_ki
            // 
            this.tb_pid_rate_pitch_roll_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_ki.Location = new System.Drawing.Point(32, 48);
            this.tb_pid_rate_pitch_roll_ki.Maximum = 255;
            this.tb_pid_rate_pitch_roll_ki.Name = "tb_pid_rate_pitch_roll_ki";
            this.tb_pid_rate_pitch_roll_ki.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_rate_pitch_roll_ki.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_ki.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_ki.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_ki_Scroll);
            // 
            // label8
            // 
            this.label8.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(6, 52);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(16, 13);
            this.label8.TabIndex = 1;
            this.label8.Text = "Ki";
            this.label8.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_rate_pitch_roll_kd
            // 
            this.tb_pid_rate_pitch_roll_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_kd.Location = new System.Drawing.Point(32, 76);
            this.tb_pid_rate_pitch_roll_kd.Maximum = 255;
            this.tb_pid_rate_pitch_roll_kd.Name = "tb_pid_rate_pitch_roll_kd";
            this.tb_pid_rate_pitch_roll_kd.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_rate_pitch_roll_kd.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_kd.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_kd.Value = 70;
            this.tb_pid_rate_pitch_roll_kd.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_kd_Scroll);
            // 
            // label9
            // 
            this.label9.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(4, 80);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(20, 13);
            this.label9.TabIndex = 1;
            this.label9.Text = "Kd";
            this.label9.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label10
            // 
            this.label10.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(106, 2);
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
            this.label7.Location = new System.Drawing.Point(4, 24);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(20, 13);
            this.label7.TabIndex = 1;
            this.label7.Text = "Kp";
            this.label7.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_rate_pitch_roll_f1
            // 
            this.tb_pid_rate_pitch_roll_f1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_f1.Location = new System.Drawing.Point(32, 104);
            this.tb_pid_rate_pitch_roll_f1.Maximum = 255;
            this.tb_pid_rate_pitch_roll_f1.Name = "tb_pid_rate_pitch_roll_f1";
            this.tb_pid_rate_pitch_roll_f1.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_rate_pitch_roll_f1.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_f1.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_f1.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_f1_Scroll);
            // 
            // tb_pid_rate_pitch_roll_f2
            // 
            this.tb_pid_rate_pitch_roll_f2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_pitch_roll_f2.Location = new System.Drawing.Point(32, 132);
            this.tb_pid_rate_pitch_roll_f2.Maximum = 255;
            this.tb_pid_rate_pitch_roll_f2.Name = "tb_pid_rate_pitch_roll_f2";
            this.tb_pid_rate_pitch_roll_f2.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_rate_pitch_roll_f2.TabIndex = 0;
            this.tb_pid_rate_pitch_roll_f2.TickFrequency = 5;
            this.tb_pid_rate_pitch_roll_f2.Value = 30;
            this.tb_pid_rate_pitch_roll_f2.Scroll += new System.EventHandler(this.tb_pid_rate_pitch_roll_f2_Scroll);
            // 
            // label11
            // 
            this.label11.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(6, 108);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(16, 13);
            this.label11.TabIndex = 1;
            this.label11.Text = "f1";
            this.label11.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label12
            // 
            this.label12.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(6, 136);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(16, 13);
            this.label12.TabIndex = 1;
            this.label12.Text = "f2";
            this.label12.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel8
            // 
            this.tableLayoutPanel8.ColumnCount = 2;
            this.tableLayoutPanel8.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10.25641F));
            this.tableLayoutPanel8.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 89.74359F));
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_kp, 1, 1);
            this.tableLayoutPanel8.Controls.Add(this.label3, 0, 1);
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_ki, 1, 2);
            this.tableLayoutPanel8.Controls.Add(this.label4, 0, 2);
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_kd, 1, 3);
            this.tableLayoutPanel8.Controls.Add(this.label5, 0, 3);
            this.tableLayoutPanel8.Controls.Add(this.label6, 1, 0);
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_f1, 1, 4);
            this.tableLayoutPanel8.Controls.Add(this.label13, 0, 4);
            this.tableLayoutPanel8.Controls.Add(this.label14, 0, 5);
            this.tableLayoutPanel8.Controls.Add(this.tb_pid_angle_pitch_roll_f2, 1, 5);
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
            this.tableLayoutPanel8.Size = new System.Drawing.Size(285, 156);
            this.tableLayoutPanel8.TabIndex = 0;
            // 
            // tb_pid_angle_pitch_roll_kp
            // 
            this.tb_pid_angle_pitch_roll_kp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_kp.Location = new System.Drawing.Point(32, 20);
            this.tb_pid_angle_pitch_roll_kp.Maximum = 255;
            this.tb_pid_angle_pitch_roll_kp.Name = "tb_pid_angle_pitch_roll_kp";
            this.tb_pid_angle_pitch_roll_kp.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_angle_pitch_roll_kp.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_kp.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_kp.Value = 36;
            this.tb_pid_angle_pitch_roll_kp.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_kp_Scroll);
            // 
            // label3
            // 
            this.label3.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(4, 24);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(20, 13);
            this.label3.TabIndex = 1;
            this.label3.Text = "Kp";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_pitch_roll_ki
            // 
            this.tb_pid_angle_pitch_roll_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_ki.Location = new System.Drawing.Point(32, 48);
            this.tb_pid_angle_pitch_roll_ki.Maximum = 255;
            this.tb_pid_angle_pitch_roll_ki.Name = "tb_pid_angle_pitch_roll_ki";
            this.tb_pid_angle_pitch_roll_ki.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_angle_pitch_roll_ki.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_ki.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_ki.Value = 100;
            this.tb_pid_angle_pitch_roll_ki.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_ki_Scroll);
            // 
            // label4
            // 
            this.label4.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 52);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(16, 13);
            this.label4.TabIndex = 1;
            this.label4.Text = "Ki";
            this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_pitch_roll_kd
            // 
            this.tb_pid_angle_pitch_roll_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_kd.Location = new System.Drawing.Point(32, 76);
            this.tb_pid_angle_pitch_roll_kd.Maximum = 255;
            this.tb_pid_angle_pitch_roll_kd.Name = "tb_pid_angle_pitch_roll_kd";
            this.tb_pid_angle_pitch_roll_kd.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_angle_pitch_roll_kd.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_kd.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_kd.Value = 80;
            this.tb_pid_angle_pitch_roll_kd.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_kd_Scroll);
            // 
            // label5
            // 
            this.label5.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(4, 80);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(20, 13);
            this.label5.TabIndex = 1;
            this.label5.Text = "Kd";
            this.label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label6
            // 
            this.label6.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(104, 2);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(105, 13);
            this.label6.TabIndex = 1;
            this.label6.Text = "PID Angle Pitch/Roll";
            this.label6.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_pitch_roll_f1
            // 
            this.tb_pid_angle_pitch_roll_f1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_f1.Location = new System.Drawing.Point(32, 104);
            this.tb_pid_angle_pitch_roll_f1.Maximum = 255;
            this.tb_pid_angle_pitch_roll_f1.Name = "tb_pid_angle_pitch_roll_f1";
            this.tb_pid_angle_pitch_roll_f1.Size = new System.Drawing.Size(250, 22);
            this.tb_pid_angle_pitch_roll_f1.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_f1.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_f1.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_f1_Scroll);
            // 
            // label13
            // 
            this.label13.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(6, 108);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(16, 13);
            this.label13.TabIndex = 1;
            this.label13.Text = "f1";
            this.label13.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label14
            // 
            this.label14.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(6, 136);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(16, 13);
            this.label14.TabIndex = 1;
            this.label14.Text = "f2";
            this.label14.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_pitch_roll_f2
            // 
            this.tb_pid_angle_pitch_roll_f2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_pitch_roll_f2.Location = new System.Drawing.Point(32, 132);
            this.tb_pid_angle_pitch_roll_f2.Maximum = 255;
            this.tb_pid_angle_pitch_roll_f2.Name = "tb_pid_angle_pitch_roll_f2";
            this.tb_pid_angle_pitch_roll_f2.Size = new System.Drawing.Size(250, 21);
            this.tb_pid_angle_pitch_roll_f2.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_f2.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_f2.Value = 95;
            this.tb_pid_angle_pitch_roll_f2.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_f2_Scroll);
            // 
            // tableLayoutPanel11
            // 
            this.tableLayoutPanel11.ColumnCount = 2;
            this.tableLayoutPanel11.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10.25641F));
            this.tableLayoutPanel11.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 89.74359F));
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_kp, 1, 1);
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_ki, 1, 2);
            this.tableLayoutPanel11.Controls.Add(this.label15, 0, 2);
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_kd, 1, 3);
            this.tableLayoutPanel11.Controls.Add(this.label16, 0, 3);
            this.tableLayoutPanel11.Controls.Add(this.label17, 1, 0);
            this.tableLayoutPanel11.Controls.Add(this.label18, 0, 1);
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_f1, 1, 4);
            this.tableLayoutPanel11.Controls.Add(this.tb_pid_rate_yaw_f2, 1, 5);
            this.tableLayoutPanel11.Controls.Add(this.label19, 0, 4);
            this.tableLayoutPanel11.Controls.Add(this.label20, 0, 5);
            this.tableLayoutPanel11.Location = new System.Drawing.Point(294, 165);
            this.tableLayoutPanel11.Name = "tableLayoutPanel11";
            this.tableLayoutPanel11.RowCount = 6;
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 17F));
            this.tableLayoutPanel11.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel11.Size = new System.Drawing.Size(296, 157);
            this.tableLayoutPanel11.TabIndex = 0;
            // 
            // tb_pid_rate_yaw_kp
            // 
            this.tb_pid_rate_yaw_kp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_kp.Location = new System.Drawing.Point(33, 20);
            this.tb_pid_rate_yaw_kp.Maximum = 255;
            this.tb_pid_rate_yaw_kp.Name = "tb_pid_rate_yaw_kp";
            this.tb_pid_rate_yaw_kp.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_rate_yaw_kp.TabIndex = 0;
            this.tb_pid_rate_yaw_kp.TickFrequency = 5;
            this.tb_pid_rate_yaw_kp.Value = 240;
            this.tb_pid_rate_yaw_kp.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_kp_Scroll);
            // 
            // tb_pid_rate_yaw_ki
            // 
            this.tb_pid_rate_yaw_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_ki.Location = new System.Drawing.Point(33, 48);
            this.tb_pid_rate_yaw_ki.Maximum = 255;
            this.tb_pid_rate_yaw_ki.Name = "tb_pid_rate_yaw_ki";
            this.tb_pid_rate_yaw_ki.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_rate_yaw_ki.TabIndex = 0;
            this.tb_pid_rate_yaw_ki.TickFrequency = 5;
            this.tb_pid_rate_yaw_ki.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_ki_Scroll);
            // 
            // label15
            // 
            this.label15.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(7, 52);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(16, 13);
            this.label15.TabIndex = 1;
            this.label15.Text = "Ki";
            this.label15.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_rate_yaw_kd
            // 
            this.tb_pid_rate_yaw_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_kd.Location = new System.Drawing.Point(33, 76);
            this.tb_pid_rate_yaw_kd.Maximum = 255;
            this.tb_pid_rate_yaw_kd.Name = "tb_pid_rate_yaw_kd";
            this.tb_pid_rate_yaw_kd.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_rate_yaw_kd.TabIndex = 0;
            this.tb_pid_rate_yaw_kd.TickFrequency = 5;
            this.tb_pid_rate_yaw_kd.Value = 120;
            this.tb_pid_rate_yaw_kd.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_kd_Scroll);
            // 
            // label16
            // 
            this.label16.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(5, 80);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(20, 13);
            this.label16.TabIndex = 1;
            this.label16.Text = "Kd";
            this.label16.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label17
            // 
            this.label17.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label17.AutoSize = true;
            this.label17.Location = new System.Drawing.Point(125, 2);
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
            this.label18.Location = new System.Drawing.Point(5, 24);
            this.label18.Name = "label18";
            this.label18.Size = new System.Drawing.Size(20, 13);
            this.label18.TabIndex = 1;
            this.label18.Text = "Kp";
            this.label18.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_rate_yaw_f1
            // 
            this.tb_pid_rate_yaw_f1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_f1.Location = new System.Drawing.Point(33, 104);
            this.tb_pid_rate_yaw_f1.Maximum = 255;
            this.tb_pid_rate_yaw_f1.Name = "tb_pid_rate_yaw_f1";
            this.tb_pid_rate_yaw_f1.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_rate_yaw_f1.TabIndex = 0;
            this.tb_pid_rate_yaw_f1.TickFrequency = 5;
            this.tb_pid_rate_yaw_f1.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_f1_Scroll);
            // 
            // tb_pid_rate_yaw_f2
            // 
            this.tb_pid_rate_yaw_f2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_rate_yaw_f2.Location = new System.Drawing.Point(33, 132);
            this.tb_pid_rate_yaw_f2.Maximum = 255;
            this.tb_pid_rate_yaw_f2.Name = "tb_pid_rate_yaw_f2";
            this.tb_pid_rate_yaw_f2.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_rate_yaw_f2.TabIndex = 0;
            this.tb_pid_rate_yaw_f2.TickFrequency = 5;
            this.tb_pid_rate_yaw_f2.Value = 30;
            this.tb_pid_rate_yaw_f2.Scroll += new System.EventHandler(this.tb_pid_rate_yaw_f2_Scroll);
            // 
            // label19
            // 
            this.label19.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label19.AutoSize = true;
            this.label19.Location = new System.Drawing.Point(7, 108);
            this.label19.Name = "label19";
            this.label19.Size = new System.Drawing.Size(16, 13);
            this.label19.TabIndex = 1;
            this.label19.Text = "f1";
            this.label19.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label20
            // 
            this.label20.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label20.AutoSize = true;
            this.label20.Location = new System.Drawing.Point(7, 136);
            this.label20.Name = "label20";
            this.label20.Size = new System.Drawing.Size(16, 13);
            this.label20.TabIndex = 1;
            this.label20.Text = "f2";
            this.label20.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel12
            // 
            this.tableLayoutPanel12.ColumnCount = 2;
            this.tableLayoutPanel12.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10.25641F));
            this.tableLayoutPanel12.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 89.74359F));
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_kp, 1, 1);
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_ki, 1, 2);
            this.tableLayoutPanel12.Controls.Add(this.label21, 0, 2);
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_kd, 1, 3);
            this.tableLayoutPanel12.Controls.Add(this.label22, 0, 3);
            this.tableLayoutPanel12.Controls.Add(this.label23, 1, 0);
            this.tableLayoutPanel12.Controls.Add(this.label24, 0, 1);
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_f1, 1, 4);
            this.tableLayoutPanel12.Controls.Add(this.tb_pid_angle_yaw_f2, 1, 5);
            this.tableLayoutPanel12.Controls.Add(this.label25, 0, 4);
            this.tableLayoutPanel12.Controls.Add(this.label26, 0, 5);
            this.tableLayoutPanel12.Location = new System.Drawing.Point(294, 3);
            this.tableLayoutPanel12.Name = "tableLayoutPanel12";
            this.tableLayoutPanel12.RowCount = 6;
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 11F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 18F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 17F));
            this.tableLayoutPanel12.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel12.Size = new System.Drawing.Size(296, 156);
            this.tableLayoutPanel12.TabIndex = 0;
            // 
            // tb_pid_angle_yaw_kp
            // 
            this.tb_pid_angle_yaw_kp.Location = new System.Drawing.Point(33, 20);
            this.tb_pid_angle_yaw_kp.Maximum = 255;
            this.tb_pid_angle_yaw_kp.Name = "tb_pid_angle_yaw_kp";
            this.tb_pid_angle_yaw_kp.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_angle_yaw_kp.TabIndex = 0;
            this.tb_pid_angle_yaw_kp.TickFrequency = 5;
            this.tb_pid_angle_yaw_kp.Value = 250;
            this.tb_pid_angle_yaw_kp.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_kp_Scroll);
            // 
            // tb_pid_angle_yaw_ki
            // 
            this.tb_pid_angle_yaw_ki.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_yaw_ki.Location = new System.Drawing.Point(33, 48);
            this.tb_pid_angle_yaw_ki.Maximum = 255;
            this.tb_pid_angle_yaw_ki.Name = "tb_pid_angle_yaw_ki";
            this.tb_pid_angle_yaw_ki.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_angle_yaw_ki.TabIndex = 0;
            this.tb_pid_angle_yaw_ki.TickFrequency = 5;
            this.tb_pid_angle_yaw_ki.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_ki_Scroll);
            // 
            // label21
            // 
            this.label21.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label21.AutoSize = true;
            this.label21.Location = new System.Drawing.Point(7, 52);
            this.label21.Name = "label21";
            this.label21.Size = new System.Drawing.Size(16, 13);
            this.label21.TabIndex = 1;
            this.label21.Text = "Ki";
            this.label21.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_yaw_kd
            // 
            this.tb_pid_angle_yaw_kd.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_yaw_kd.Location = new System.Drawing.Point(33, 76);
            this.tb_pid_angle_yaw_kd.Maximum = 255;
            this.tb_pid_angle_yaw_kd.Name = "tb_pid_angle_yaw_kd";
            this.tb_pid_angle_yaw_kd.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_angle_yaw_kd.TabIndex = 0;
            this.tb_pid_angle_yaw_kd.TickFrequency = 5;
            this.tb_pid_angle_yaw_kd.Value = 255;
            this.tb_pid_angle_yaw_kd.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_kd_Scroll);
            // 
            // label22
            // 
            this.label22.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label22.AutoSize = true;
            this.label22.Location = new System.Drawing.Point(5, 80);
            this.label22.Name = "label22";
            this.label22.Size = new System.Drawing.Size(20, 13);
            this.label22.TabIndex = 1;
            this.label22.Text = "Kd";
            this.label22.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label23
            // 
            this.label23.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label23.AutoSize = true;
            this.label23.Location = new System.Drawing.Point(123, 2);
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
            this.label24.Location = new System.Drawing.Point(5, 24);
            this.label24.Name = "label24";
            this.label24.Size = new System.Drawing.Size(20, 13);
            this.label24.TabIndex = 1;
            this.label24.Text = "Kp";
            this.label24.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_yaw_f1
            // 
            this.tb_pid_angle_yaw_f1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_yaw_f1.Location = new System.Drawing.Point(33, 104);
            this.tb_pid_angle_yaw_f1.Maximum = 255;
            this.tb_pid_angle_yaw_f1.Name = "tb_pid_angle_yaw_f1";
            this.tb_pid_angle_yaw_f1.Size = new System.Drawing.Size(260, 22);
            this.tb_pid_angle_yaw_f1.TabIndex = 0;
            this.tb_pid_angle_yaw_f1.TickFrequency = 5;
            this.tb_pid_angle_yaw_f1.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_f1_Scroll);
            // 
            // tb_pid_angle_yaw_f2
            // 
            this.tb_pid_angle_yaw_f2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tb_pid_angle_yaw_f2.Location = new System.Drawing.Point(33, 132);
            this.tb_pid_angle_yaw_f2.Maximum = 255;
            this.tb_pid_angle_yaw_f2.Name = "tb_pid_angle_yaw_f2";
            this.tb_pid_angle_yaw_f2.Size = new System.Drawing.Size(260, 21);
            this.tb_pid_angle_yaw_f2.TabIndex = 0;
            this.tb_pid_angle_yaw_f2.TickFrequency = 5;
            this.tb_pid_angle_yaw_f2.Value = 95;
            this.tb_pid_angle_yaw_f2.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_f2_Scroll);
            // 
            // label25
            // 
            this.label25.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label25.AutoSize = true;
            this.label25.Location = new System.Drawing.Point(7, 108);
            this.label25.Name = "label25";
            this.label25.Size = new System.Drawing.Size(16, 13);
            this.label25.TabIndex = 1;
            this.label25.Text = "f1";
            this.label25.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label26
            // 
            this.label26.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label26.AutoSize = true;
            this.label26.Location = new System.Drawing.Point(7, 136);
            this.label26.Name = "label26";
            this.label26.Size = new System.Drawing.Size(16, 13);
            this.label26.TabIndex = 1;
            this.label26.Text = "f2";
            this.label26.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel13
            // 
            this.tableLayoutPanel13.ColumnCount = 1;
            this.tableLayoutPanel13.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel13.Controls.Add(this.tb_pid_angle_pitch_roll_out_filter, 0, 1);
            this.tableLayoutPanel13.Controls.Add(this.label27, 0, 0);
            this.tableLayoutPanel13.Controls.Add(this.label28, 0, 3);
            this.tableLayoutPanel13.Controls.Add(this.tb_pid_angle_yaw_out_filter, 0, 4);
            this.tableLayoutPanel13.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel13.Location = new System.Drawing.Point(596, 3);
            this.tableLayoutPanel13.Name = "tableLayoutPanel13";
            this.tableLayoutPanel13.RowCount = 6;
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 54.05405F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 45.94595F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 10F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 17F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 24F));
            this.tableLayoutPanel13.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 49F));
            this.tableLayoutPanel13.Size = new System.Drawing.Size(284, 156);
            this.tableLayoutPanel13.TabIndex = 1;
            // 
            // tb_pid_angle_pitch_roll_out_filter
            // 
            this.tb_pid_angle_pitch_roll_out_filter.Location = new System.Drawing.Point(3, 33);
            this.tb_pid_angle_pitch_roll_out_filter.Maximum = 255;
            this.tb_pid_angle_pitch_roll_out_filter.Name = "tb_pid_angle_pitch_roll_out_filter";
            this.tb_pid_angle_pitch_roll_out_filter.Size = new System.Drawing.Size(264, 19);
            this.tb_pid_angle_pitch_roll_out_filter.TabIndex = 0;
            this.tb_pid_angle_pitch_roll_out_filter.TickFrequency = 5;
            this.tb_pid_angle_pitch_roll_out_filter.Value = 88;
            this.tb_pid_angle_pitch_roll_out_filter.Scroll += new System.EventHandler(this.tb_pid_angle_pitch_roll_out_filter_Scroll);
            // 
            // label27
            // 
            this.label27.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label27.AutoSize = true;
            this.label27.Location = new System.Drawing.Point(67, 8);
            this.label27.Name = "label27";
            this.label27.Size = new System.Drawing.Size(150, 13);
            this.label27.TabIndex = 2;
            this.label27.Text = "PID Angle Pitch/Roll Out Filter";
            this.label27.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label28
            // 
            this.label28.Anchor = System.Windows.Forms.AnchorStyles.None;
            this.label28.AutoSize = true;
            this.label28.Location = new System.Drawing.Point(80, 67);
            this.label28.Name = "label28";
            this.label28.Size = new System.Drawing.Size(124, 13);
            this.label28.TabIndex = 2;
            this.label28.Text = "PID Angle Yaw Out Filter";
            this.label28.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tb_pid_angle_yaw_out_filter
            // 
            this.tb_pid_angle_yaw_out_filter.Location = new System.Drawing.Point(3, 85);
            this.tb_pid_angle_yaw_out_filter.Maximum = 255;
            this.tb_pid_angle_yaw_out_filter.Name = "tb_pid_angle_yaw_out_filter";
            this.tb_pid_angle_yaw_out_filter.Size = new System.Drawing.Size(264, 18);
            this.tb_pid_angle_yaw_out_filter.TabIndex = 0;
            this.tb_pid_angle_yaw_out_filter.TickFrequency = 5;
            this.tb_pid_angle_yaw_out_filter.Value = 30;
            this.tb_pid_angle_yaw_out_filter.Scroll += new System.EventHandler(this.tb_pid_angle_yaw_out_filter_Scroll);
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
            // bwUdpTransmit
            // 
            this.bwUdpTransmit.DoWork += new System.ComponentModel.DoWorkEventHandler(this.bwUdpTransmit_DoWork);
            // 
            // Ground_Station
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1284, 911);
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
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_hScale)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDown_vScale)).EndInit();
            this.tableLayoutPanelDataTx.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel7.ResumeLayout(false);
            this.tableLayoutPanel9.ResumeLayout(false);
            this.tableLayoutPanel9.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_kd)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_f1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_pitch_roll_f2)).EndInit();
            this.tableLayoutPanel8.ResumeLayout(false);
            this.tableLayoutPanel8.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_kd)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_f1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_f2)).EndInit();
            this.tableLayoutPanel11.ResumeLayout(false);
            this.tableLayoutPanel11.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_kd)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_f1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_rate_yaw_f2)).EndInit();
            this.tableLayoutPanel12.ResumeLayout(false);
            this.tableLayoutPanel12.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kp)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_ki)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_kd)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_f1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_f2)).EndInit();
            this.tableLayoutPanel13.ResumeLayout(false);
            this.tableLayoutPanel13.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_pitch_roll_out_filter)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tb_pid_angle_yaw_out_filter)).EndInit();
            this.tableLayoutPanel10.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.numericUpDownDataTx)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.ComponentModel.BackgroundWorker bwUdpSniffer;
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
        private System.Windows.Forms.TrackBar tb_pid_angle_pitch_roll_f1;
        private System.Windows.Forms.TrackBar tb_pid_rate_pitch_roll_f1;
        private System.Windows.Forms.TrackBar tb_pid_rate_pitch_roll_f2;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.TrackBar tb_pid_rate_pitch_roll_ki;
        private System.Windows.Forms.TrackBar tb_pid_angle_pitch_roll_f2;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel11;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_kp;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_ki;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_kd;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.Label label18;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_f1;
        private System.Windows.Forms.TrackBar tb_pid_rate_yaw_f2;
        private System.Windows.Forms.Label label19;
        private System.Windows.Forms.Label label20;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel12;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_kp;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_ki;
        private System.Windows.Forms.Label label21;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_kd;
        private System.Windows.Forms.Label label22;
        private System.Windows.Forms.Label label23;
        private System.Windows.Forms.Label label24;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_f1;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_f2;
        private System.Windows.Forms.Label label25;
        private System.Windows.Forms.Label label26;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel13;
        private System.Windows.Forms.TrackBar tb_pid_angle_pitch_roll_out_filter;
        private System.Windows.Forms.Label label27;
        private System.Windows.Forms.Label label28;
        private System.Windows.Forms.TrackBar tb_pid_angle_yaw_out_filter;
    }
}

