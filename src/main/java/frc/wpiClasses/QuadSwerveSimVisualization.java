package frc.wpiClasses;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class QuadSwerveSimVisualization {
    
    List<SwerveModuleSim> m_modules;

    Mechanism2d m_modViz;
    List<MechanismRoot2d> m_modRoots;

    List<MechanismLigament2d> m_modAzmthAngInd;
    List<MechanismLigament2d> m_modWheelSpdInd;


    Field2d m_chassisViz;
    FieldObject2d m_chassisObj;


    public QuadSwerveSimVisualization(
        double wheelBaseWidthM,
        double wheelBaseLengthM,
        List<SwerveModuleSim> modules
    ) {
        double vizWidth = wheelBaseWidthM*1.5;
        double vizHeight = wheelBaseLengthM*1.5;
        double vizCenterX = vizWidth/2;
        double vizCenterY = vizHeight/2;
        
        double modCenterDeltaX = vizWidth/5;
        double modCenterDeltaY = vizHeight/5;


        m_modViz = new Mechanism2d(vizWidth, vizHeight);
        m_modRoots = Arrays.asList(
            m_modViz.getRoot("FL", vizCenterX - modCenterDeltaX, vizCenterY + modCenterDeltaY),
            m_modViz.getRoot("FR", vizCenterX + modCenterDeltaX, vizCenterY + modCenterDeltaY),
            m_modViz.getRoot("BL", vizCenterX - modCenterDeltaX, vizCenterY - modCenterDeltaY),
            m_modViz.getRoot("BR", vizCenterX + modCenterDeltaX, vizCenterY - modCenterDeltaY)
        );

        double azmthIndLen = Math.sqrt(wheelBaseWidthM*wheelBaseWidthM + wheelBaseLengthM * wheelBaseLengthM) * 0.15;

        m_modAzmthAngInd = Arrays.asList(
            m_modRoots.get(0).append(new MechanismLigament2d("FL_azmth", azmthIndLen, 0 , 5, new Color8Bit(Color.kRed))),
            m_modRoots.get(1).append(new MechanismLigament2d("FR_azmth", azmthIndLen, 10, 5, new Color8Bit(Color.kGreen))),
            m_modRoots.get(2).append(new MechanismLigament2d("BL_azmth", azmthIndLen, 20, 5, new Color8Bit(Color.kBlue))),
            m_modRoots.get(3).append(new MechanismLigament2d("BR_azmth", azmthIndLen, 30, 5, new Color8Bit(Color.kOrange)))
        );

        m_modules = modules;

        m_chassisViz = new Field2d();
        m_chassisObj = m_chassisViz.getObject("Sim Chassis");

        //Add robot bounding box and direction arrow
        double bumperDeltaX = wheelBaseWidthM/2 + azmthIndLen;
        double bumperDeltaY = wheelBaseLengthM/2 + azmthIndLen;
        m_modViz.getRoot("BumperCornerFL", vizCenterX - bumperDeltaX, vizCenterY + bumperDeltaY).append(new MechanismLigament2d("Front Bumper", bumperDeltaX*2 , 0,   3, new Color8Bit(Color.kGray)));
        m_modViz.getRoot("BumperCornerFR", vizCenterX + bumperDeltaX, vizCenterY + bumperDeltaY).append(new MechanismLigament2d("Right Bumper", bumperDeltaY*2 , -90, 3, new Color8Bit(Color.kGray)));
        m_modViz.getRoot("BumperCornerBL", vizCenterX - bumperDeltaX, vizCenterY - bumperDeltaY).append(new MechanismLigament2d("Left Bumper",  bumperDeltaY*2 , 90,  3, new Color8Bit(Color.kGray)));
        m_modViz.getRoot("BumperCornerBR", vizCenterX + bumperDeltaX, vizCenterY - bumperDeltaY).append(new MechanismLigament2d("Rear Bumper",  bumperDeltaX*2 , 180, 3, new Color8Bit(Color.kGray)));

        double arrowSegLen = wheelBaseLengthM/10;
        m_modViz.getRoot("arrowPt1", vizCenterX, vizCenterY - arrowSegLen).append(new MechanismLigament2d("Rear Bumper",  arrowSegLen*2 , 90, 3, new Color8Bit(Color.kGray)));
        m_modViz.getRoot("arrowPt2", vizCenterX, vizCenterY + arrowSegLen).append(new MechanismLigament2d("Rear Bumper",  arrowSegLen   , 225, 3, new Color8Bit(Color.kGray)));
        m_modViz.getRoot("arrowPt3", vizCenterX, vizCenterY + arrowSegLen).append(new MechanismLigament2d("Rear Bumper",  arrowSegLen   , -45, 3, new Color8Bit(Color.kGray)));

        

        SmartDashboard.putData("SwerveSimModuleVisualization", m_modViz);
        SmartDashboard.putData("Field", m_chassisViz);
        
    }

    public void update(Pose2d chassisPose){

        for(int i = 0; i < 4; i++){
            m_modAzmthAngInd.get(i).setAngle(m_modules.get(i).getAzimuthEncoderPositionRev()*360.0 + 90);
        }


        m_chassisObj.setPose(chassisPose);
    }

}
