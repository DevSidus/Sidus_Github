using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ground_Station
{
    class cDataTxDisplayClass
    {
        public void init(ListView lv, structMsgUdpT01 data)
        {
            lv.DoubleBuffered(true);

            foreach (var prop in data.GetType().GetProperties())
            {
                ListViewItem lvi = new ListViewItem(prop.Name);
                lvi.SubItems.Add(prop.GetValue(data, null).ToString());
                lv.Items.Add(lvi);
            }
         
        }
        public void update(ListView lv, structMsgUdpT01 data)
        {

            foreach (var prop in data.GetType().GetProperties())
            {
                ListViewItem lvi = lv.FindItemWithText(prop.Name);
                if (lvi != null)
                {
                    lvi.SubItems[1].Text = string.Format("{0,8:0.00}", Convert.ToDouble(prop.GetValue(data, null).ToString()));
                }
            }

        }

        public bool setData(ListView lv, ref structMsgUdpT01 message, double data)
        {

            object tempObj = message;
            foreach (var prop in tempObj.GetType().GetProperties())
            {
                ListViewItem lvi = lv.FindItemWithText(prop.Name);
                if(lvi!=null && lvi.Checked)
                {
                    try
                    {
                        prop.SetValue(tempObj, Convert.ChangeType(data, prop.PropertyType), null);
                    }
                    catch
                    {
                        return false;
                    }
                }    
            }
            message = (structMsgUdpT01)tempObj;
            return true;
        }
    }
}
