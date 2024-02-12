package uuspaceagent;

public class Voxel {
    public byte label;
    public DPos3 pos;

    public Voxel(DPos3 pos) {
        this.pos = pos;
        label = Label.OPEN;
    }
}
