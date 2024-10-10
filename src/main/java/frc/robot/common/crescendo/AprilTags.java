package frc.robot.common.crescendo;

public enum AprilTags {
    BLUE_SOURCE_LEFT,
    BLUE_SOURCE_RIGHT,

    RED_SPEAKER_SOURCE_SIDE,
    RED_SPEAKER_CENTER,

    RED_AMP,

    BLUE_AMP,

    BLUE_SPEAKER_CENTER,
    BLUE_SPEAKER_SOURCE_SIDE,

    RED_SOURCE_LEFT,
    RED_SOURCE_RIGHT,

    RED_STAGE_LEFT,
    RED_STAGE_RIGHT,
    RED_STAGE_CENTER,

    BLUE_STAGE_CENTER,
    BLUE_STAGE_LEFT,
    BLUE_STAGE_RIGHT;

    public int id() {
        return ordinal() + 1;
    }
}