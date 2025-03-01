package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot.Container;

public class LedManager extends SubsystemBase {

    private record Layer(PatternType patternType, int priority) {

        public interface PatternType {
            public double getValue();
        }

        public enum FixedPalletePatternType implements PatternType {
            Rainbow(-0.99),
            RainbowParty(-0.97),
            RainbowOcean(-0.95),
            RainbowLava(-0.93),
            RainbowForest(-0.91),
            RainbowGlitter(-0.89),
            Confetti(-0.87),
            ShotRed(-0.85),
            ShotBlue(-0.83),
            ShotWhite(-0.81),
            SinelonRainbow(-0.79),
            SinelonParty(-0.77),
            SinelonOcean(-0.75),
            SinelonLava(-0.73),
            SinelonForest(-0.71),
            BeatsPerMinuteRainbow(-0.69),
            BeatsPerMinuteParty(-0.67),
            BeatsPerMinuteOcean(-0.65),
            BeatsPerMinuteLava(-0.63),
            BeatsPerMinuteForest(-0.61),
            FireMedium(-0.59),
            FireLarge(-0.57),
            TwinklesRainbow(-0.55),
            TwinklesParty(-0.53),
            TwinklesOcean(-0.51),
            TwinklesLava(-0.49),
            TwinklesForest(-0.47),
            ColorWavesRainbow(-0.45),
            ColorWavesParty(-0.43),
            ColorWavesOcean(-0.41),
            ColorWavesLava(-0.39),
            ColorWavesForest(-0.37),
            LarsonScannerRed(-0.35),
            LarsonScannerGray(-0.33),
            LightChaseRed(-0.31),
            LightChaseBlue(-0.29),
            LightChaseGray(-0.27),
            HeartbeatRed(-0.25),
            HeartbeatBlue(-0.23),
            HeartbeatWhite(-0.21),
            HeartbeatGray(-0.19),
            BreathRed(-0.17),
            BreathBlue(-0.15),
            BreathGray(-0.13),
            StrobeRed(-0.11),
            StrobeBlue(-0.09),
            StrobeGold(-0.07),
            StrobeWhite(-0.05);

            private final double value;

            FixedPalletePatternType(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }

        public enum Color1PatternType implements PatternType {
            EndToEndBlendToBlack(-0.03),
            LarsonScanner(-0.01),
            LightChase(0.01),
            HeartbeatSlow(0.03),
            HeartbeatMedium(0.05),
            HeartbeatFast(0.07),
            BreathSlow(0.09),
            BreathFast(0.11),
            Shot(0.13),
            Strobe(0.15);

            private final double value;

            Color1PatternType(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }

        public enum Color2PatternType implements PatternType {
            EndToEndBlendToBlack(0.17),
            LarsonScanner(0.19),
            LightChase(0.21),
            HeartbeatSlow(0.23),
            HeartbeatMedium(0.25),
            HeartbeatFast(0.27),
            BreathSlow(0.29),
            BreathFast(0.31),
            Shot(0.33),
            Strobe(0.35);

            private final double value;

            Color2PatternType(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }

        public enum Color1And2PatternType implements PatternType {
            Sparkle1On2(0.37),
            Sparkle2On1(0.39),
            Gradient1And2(0.41),
            BeatsPerMinute1And2(0.43),
            EndToEndBlend1To2(0.45),
            EndToEndBlend(0.47),
            Color1And2NoBlend(0.49),
            Twinkles(0.51),
            ColorWaves(0.53),
            Sinelon1And2(0.55);

            private final double value;

            Color1And2PatternType(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }

        public enum SolidColorType implements PatternType {
            HotPink(0.57),
            DarkRed(0.59),
            Red(0.61),
            RedOrange(0.63),
            Orange(0.65),
            Gold(0.67),
            Yellow(0.69),
            LawnGreen(0.71),
            Lime(0.73),
            DarkGreen(0.75),
            Green(0.77),
            BlueGreen(0.79),
            Aqua(0.81),
            SkyBlue(0.83),
            DarkBlue(0.85),
            Blue(0.87),
            BlueViolet(0.89),
            Violet(0.91),
            White(0.93),
            Gray(0.95),
            DarkGray(0.97),
            Black(0.99);

            private final double value;

            SolidColorType(double value) {
                this.value = value;
            }

            public double getValue() {
                return value;
            }
        }
    }

    Container.Subsystems subsystems;
    ArrayList<Layer> layers = new ArrayList<Layer>();

    Spark pwm;

    SimpleWidget widget;
    GenericEntry widgetEntry;

    double lastStrobeTimestamp = 0;
    Layer.PatternType lastStrobePattern = Layer.FixedPalletePatternType.ColorWavesLava; // dummy value

    public LedManager(Container.Subsystems subsystems, int PWMPort) {
        this.subsystems = subsystems;
        pwm = new Spark(PWMPort);
        widget = Shuffleboard.getTab("SmartDashboard").add("LED status", false);
        widgetEntry = widget.getEntry();
    }


    @Override
    public void periodic() {
        //#region actual layer management
        {
            //Low Priority
            if (subsystems.coralIntake.state == CoralIntake.States.IDLE) {
                layers.add(new Layer(Layer.SolidColorType.DarkRed, -69));
            }
            if (subsystems.coralIntake.state == CoralIntake.States.WANTS_CORAL) {
                layers.add(new Layer(Layer.FixedPalletePatternType.StrobeBlue, 2));
            }
            if (subsystems.coralIntake.state == CoralIntake.States.HAS_CORAL) {
                layers.add(new Layer(Layer.SolidColorType.DarkGreen, 3));
            }


            //Medium Priority


            //High Priority
        }


        //#endregion layer management

        //collect and display layers
        if (layers.isEmpty()) {
            return;
        }

        int highestPriority = Integer.MIN_VALUE;
        int highestPriorityIndex = 0;
        for (int i = 0; i < layers.size(); i++) {
            if (layers.get(i).priority > highestPriority) {
                highestPriority = layers.get(i).priority;
                highestPriorityIndex = i;
            }
        }

        Layer currentLayer = layers.get(highestPriorityIndex);
        pwm.set(currentLayer.patternType.getValue());

        if (currentLayer.patternType instanceof Layer.SolidColorType color) {
            widget.withProperties(Map.of("colorWhenTrue", mapToColorName(color)));
            widgetEntry.setBoolean(true);
            lastStrobeTimestamp = 0;
        }

        if (currentLayer.patternType instanceof Layer.FixedPalletePatternType color) {
            if (lastStrobeTimestamp == 0 && lastStrobePattern != color) {
                    var res = mapDualColorNames(color);
                    res.ifPresent(map -> widget.withProperties(map));
                    lastStrobePattern = color;
            }

            double timestamp = Timer.getFPGATimestamp();
            if (timestamp - lastStrobeTimestamp > 0.5) {
                widgetEntry.setBoolean(!widgetEntry.getBoolean(false));
                lastStrobeTimestamp = timestamp;
            }
        }

        layers.clear();
    }

    String mapToColorName(Layer.SolidColorType pattern) {
        return switch (pattern) {
            case HotPink -> "Hot Pink";
            case DarkRed -> "Dark Red";
            case Red -> "Red";
            case RedOrange -> "Red Orange";
            case Orange -> "Orange";
            case Gold -> "Gold";
            case Yellow -> "Yellow";
            case LawnGreen -> "Lawn Green";
            case Lime -> "Lime";
            case DarkGreen -> "Dark Green";
            case Green -> "Green";
            case BlueGreen -> "Blue Green";
            case Aqua -> "Aqua";
            case SkyBlue -> "Sky Blue";
            case DarkBlue -> "Dark Blue";
            case Blue -> "Blue";
            case BlueViolet -> "Blue Violet";
            case Violet -> "Violet";
            case White -> "White";
            case Gray -> "Gray";
            case DarkGray -> "Dark Gray";
            case Black -> "Black";
            default -> "Black";
        };
    }

    Optional<Map<String, Object>> mapDualColorNames(Layer.FixedPalletePatternType pattern) {
        return switch(pattern) {
            case StrobeBlue -> Optional.of(Map.of("colorWhenTrue", "Blue", "colorWhenFalse", "Black"));
            // to lazy to implement unused colors
            default -> Optional.empty();
        }
    }

}

