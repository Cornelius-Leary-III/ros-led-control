#include "LedControlMainWindow.h"
#include "ui_LedControlMainWindow.h"

LedControlMainWindow::LedControlMainWindow(int argc, char** argv, QWidget* parent)
  : QMainWindow(parent),
    mUi(new Ui::LedControlMainWindow),
    mLedControlNode(new LedControlNode(argc, argv)),
    mArgC(argc),
    mArgV(argv),
    mPulsePeriodCurrentText(),
    mPulseToggleStepsCurrentText(),
    mPulseOffStepsCurrentText(),
    mBehaviorCurrentIndex(-1),
    mGroupBehaviorCurrentIndex(-1)
{
   mUi->setupUi(this);

   connect(mUi->lineEditPulsePeriod,
           &QLineEdit::editingFinished,
           this,
           &LedControlMainWindow::onLineEditPulsePeriodChanged);

   connect(mUi->lineEditPulseToggleSteps,
           &QLineEdit::editingFinished,
           this,
           &LedControlMainWindow::onLineEditPulseToggleStepsChanged);

   connect(mUi->lineEditPulseOffSteps,
           &QLineEdit::editingFinished,
           this,
           &LedControlMainWindow::onLineEditPulseOffStepsChanged);

   connect(mUi->pushButtonTurnOffLeds,
           &QPushButton::clicked,
           this,
           &LedControlMainWindow::turnOffAllLeds);

   connect(mUi->pushButtonSendControlSetup,
           &QPushButton::clicked,
           this,
           &LedControlMainWindow::sendLedControlSetupMsg);

   connect(mUi->pushButtonSendControl,
           &QPushButton::clicked,
           this,
           &LedControlMainWindow::sendCurrentLedControlMsg);

   connect(mUi->comboBoxBehavior,
           QOverload<int>::of(&QComboBox::currentIndexChanged),
           this,
           &LedControlMainWindow::onComboBoxBehaviorSelected);

   connect(mUi->comboBoxGroupBehavior,
           QOverload<int>::of(&QComboBox::currentIndexChanged),
           this,
           &LedControlMainWindow::onComboBoxGroupBehaviorSelected);

   if (mLedControlNode == nullptr)
   {
      // throw exception?
      return;
   }

   connect(this,
           &LedControlMainWindow::pulsePeriodChanged,
           mLedControlNode,
           &LedControlNode::onPulsePeriodChanged);

   connect(this,
           &LedControlMainWindow::pulseToggleStepsChanged,
           mLedControlNode,
           &LedControlNode::onPulseToggleStepsChanged);

   connect(this,
           &LedControlMainWindow::pulseOffStepsChanged,
           mLedControlNode,
           &LedControlNode::onPulseOffStepsChanged);

   connect(this,
           &LedControlMainWindow::currentBehaviorChanged,
           mLedControlNode,
           &LedControlNode::onBehaviorSelected);

   connect(this,
           &LedControlMainWindow::currentGroupBehaviorChanged,
           mLedControlNode,
           &LedControlNode::onGroupBehaviorSelected);

   connect(this,
           &LedControlMainWindow::sendLedControlSetupMsg,
           mLedControlNode,
           &LedControlNode::onSendLedControlSetupMsg);

   connect(this,
           &LedControlMainWindow::sendCurrentLedControlMsg,
           mLedControlNode,
           &LedControlNode::onSendCurrentLedControlMsg);

   connect(this,
           &LedControlMainWindow::turnOffAllLeds,
           mLedControlNode,
           &LedControlNode::onTurnOffAllLeds);

   mLedControlNode->start();
}

LedControlMainWindow::~LedControlMainWindow()
{
   if (mUi != nullptr)
   {
      delete mUi;
      mUi = nullptr;
   }

   if (mLedControlNode != nullptr)
   {
      mLedControlNode->quit();
      mLedControlNode->wait();
   }
}

void LedControlMainWindow::onLineEditPulsePeriodChanged()
{
   if (mUi == nullptr || mUi->lineEditPulsePeriod == nullptr)
   {
      return;
   }

   QString text = mUi->lineEditPulsePeriod->text();

   // perform any additional validation steps.
   if (text.isEmpty())
   {
      return;
   }

   if (text != mPulsePeriodCurrentText)
   {
      emit pulsePeriodChanged(text);

      mPulsePeriodCurrentText = text;
   }
}

void LedControlMainWindow::onLineEditPulseToggleStepsChanged()
{
   if (mUi == nullptr || mUi->lineEditPulseToggleSteps == nullptr)
   {
      return;
   }

   QString text = mUi->lineEditPulseToggleSteps->text();

   // perform any additional validation steps.
   if (text.isEmpty())
   {
      return;
   }

   if (text != mPulseToggleStepsCurrentText)
   {
      emit pulseToggleStepsChanged(text);

      mPulseToggleStepsCurrentText = text;
   }
}

void LedControlMainWindow::onLineEditPulseOffStepsChanged()
{
   if (mUi == nullptr || mUi->lineEditPulseOffSteps == nullptr)
   {
      return;
   }

   QString text = mUi->lineEditPulseOffSteps->text();

   // perform any additional validation steps.
   if (text.isEmpty())
   {
      return;
   }

   if (text != mPulseOffStepsCurrentText)
   {
      emit pulseOffStepsChanged(text);

      mPulseOffStepsCurrentText = text;
   }
}

void LedControlMainWindow::onComboBoxBehaviorSelected(int index)
{
   // perform any additional validation steps.
   if (index < 0)
   {
      return;
   }

   if (index != mBehaviorCurrentIndex)
   {
      emit currentBehaviorChanged(index);

      mBehaviorCurrentIndex = index;
   }
}

void LedControlMainWindow::onComboBoxGroupBehaviorSelected(int index)
{
   // perform any additional validation steps.
   if (index < 0)
   {
      return;
   }

   if (index != mGroupBehaviorCurrentIndex)
   {
      emit currentGroupBehaviorChanged(index);

      mGroupBehaviorCurrentIndex = index;
   }
}
