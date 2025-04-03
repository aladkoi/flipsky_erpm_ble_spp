B4A=true
Group=Default Group
ModulesStructureVersion=1
Type=Class
Version=12.8
@EndOfDesignText@
Sub Class_Globals
	Private Root As B4XView 'ignore
	Private xui As XUI 'ignore
	Private CustomListView1 As CustomListView
	Dim hh As Int
	hh=45dip
End Sub

'You can add more parameters here.
Public Sub Initialize As Object
	
	Return Me
End Sub


Private Sub GetPanel(txt As String) As Panel
	Private b1 As Button
	Private lbl1 As Label
	Dim p As B4XView = xui.CreatePanel("")
	p.SetLayoutAnimated(0, 0, 0, 100%x, hh)
	If txt="" Then Return p
	lbl1.Initialize("lbl")
	lbl1.Gravity = Bit.Or(Gravity.CENTER_VERTICAL, Gravity.LEFT)
	lbl1.Text = txt
	lbl1.TextSize = 18
	lbl1.TextColor = Colors.Black
	b1.Initialize("button1")
	b1.TextSize=20
	b1.Text="Сохранить"
	b1.TextColor=Colors.Black
	b1.Color=Colors.Green
	p.AddView(lbl1, 2%x, 0, 80%x, hh)
	p.AddView(b1,85%x, 5dip,14%x,10%y) '- кнопка "+"
	Return p
End Sub

'This event will be called once, before the page becomes visible.
Private Sub B4XPage_Created (Root1 As B4XView)
	Dim p  As Panel
	Root = Root1
	'load the layout to Root
	Root.LoadLayout("SaveSpeed")
	For i=0 To 5
		p=GetPanel("Круиз "&i)
		CustomListView1.Add(p,i)
	Next
End Sub

Sub Button_Click(Value As Object)
	Dim index As Int = CustomListView1.GetItemFromView(Value)
	index=CustomListView1.GetValue(index)
	Msgbox2Async("Вы хотите сохранить на уровень круиза "&index&"?", "Вопрос",  "Да", "Нет", "", Null, False)
	Wait For Msgbox_Result (Result As Int)
	If Result = DialogResponse.POSITIVE Then
		Dim Buffer(5) As Byte
		For i=0 To Buffer.Length-1
			Buffer(i)=0
		Next
		Buffer(0)=128
		B4XPages.MainPage.SendData(Buffer)
	End If
End Sub

'You can see the list of page related events in the B4XPagesManager object. The event name is B4XPage.


