package frc.robot.Autonomous;
import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class DashboardCmd extends Command{
    private String question;
    private SendableChooser<Command> chooser = new SendableChooser<>();
    private Command selecteCommand;

    public DashboardCmd(String question, HashMap<String, Command> options){
        options.forEach((k,v) -> {
            chooser.setDefaultOption(k,v);
        });
        this.question = question;
    }

    public SendableChooser<Command> getChooser(){
        return chooser;
    }
    public String getQuestion(){
        return question;
    }

    @Override
    public void initialize(){
        try{
            selecteCommand = chooser.getSelected();
            selecteCommand.schedule();
        }catch (Exception e){
            System.out.println(question + ": No command selected");
        }
    }

    @Override
    public boolean isFinished(){
        return selecteCommand.isFinished();
    }
}