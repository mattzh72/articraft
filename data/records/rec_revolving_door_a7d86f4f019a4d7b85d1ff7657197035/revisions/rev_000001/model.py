from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolving_door")

    # Materials
    mirror_stainless = Material("mirror_stainless", rgba=(0.85, 0.85, 0.88, 1.0))
    glass_mat = Material("glass", rgba=(0.7, 0.8, 0.9, 0.3))

    # Dimensions
    drum_radius = 1.15
    drum_thickness = 0.05
    drum_height = 2.2
    opening_width = 1.05

    # 1. Drum Walls and Frame
    drum_outer = cq.Workplane("XY").cylinder(drum_height, drum_radius + drum_thickness).translate((0, 0, drum_height / 2))
    drum_inner_cut = cq.Workplane("XY").cylinder(drum_height + 0.1, drum_radius).translate((0, 0, drum_height / 2))
    drum_walls = drum_outer.cut(drum_inner_cut)

    openings_cut = cq.Workplane("XY").box(opening_width, 4.0, drum_height + 0.1).translate((0, 0, drum_height / 2))
    drum_frame_base = drum_walls.cut(openings_cut)

    # Cut out the window openings in the walls
    window_inner = (
        cq.Workplane("XY")
        .cylinder(drum_height - 0.2, drum_radius + drum_thickness + 0.1)
        .cut(cq.Workplane("XY").cylinder(drum_height - 0.2, drum_radius - 0.1))
    ).translate((0, 0, drum_height / 2))
    window_cut_box = cq.Workplane("XY").box(opening_width + 0.2, 4.0, drum_height + 0.1).translate((0, 0, drum_height / 2))
    window_cut = window_inner.cut(window_cut_box)

    drum_metal = drum_frame_base.cut(window_cut)

    # Intersect to get the glass panels
    glass_cylinder = (
        cq.Workplane("XY")
        .cylinder(drum_height, drum_radius + drum_thickness / 2 + 0.01)
        .cut(cq.Workplane("XY").cylinder(drum_height, drum_radius + drum_thickness / 2 - 0.01))
    ).translate((0, 0, drum_height / 2))
    drum_glass = glass_cylinder.intersect(window_cut)

    # 2. Canopy
    canopy_height = 0.2
    canopy = cq.Workplane("XY").cylinder(canopy_height, drum_radius + drum_thickness).translate((0, 0, drum_height + canopy_height / 2))
    drum_metal = drum_metal.union(canopy)

    # 3. Pivot mounts and Floor
    post_radius = 0.05
    drum_floor = cq.Workplane("XY").cylinder(0.02, drum_radius + drum_thickness).translate((0, 0, 0.01))
    top_pivot = cq.Workplane("XY").cylinder(0.02, post_radius + 0.05).translate((0, 0, drum_height - 0.01))
    drum_metal = drum_metal.union(drum_floor).union(top_pivot)

    # 4. Rotor Parts
    post_height = drum_height
    post = cq.Workplane("XY").cylinder(post_height, post_radius).translate((0, 0, drum_height / 2))

    wing_height = drum_height - 0.04
    wing_width = drum_radius - post_radius - 0.02
    wing_thickness = 0.04
    frame_width = 0.08

    wing_center_x = post_radius + wing_width / 2

    wing_outer = cq.Workplane("XY").box(wing_width, wing_thickness, wing_height)
    wing_window_cut = cq.Workplane("XY").box(
        wing_width - 2 * frame_width,
        wing_thickness + 0.1,
        wing_height - 2 * frame_width
    )

    wing_frame = wing_outer.cut(wing_window_cut).translate((wing_center_x, 0, drum_height / 2))
    wing_glass_panel = cq.Workplane("XY").box(
        wing_width - 2 * frame_width,
        0.01,
        wing_height - 2 * frame_width
    ).translate((wing_center_x, 0, drum_height / 2))

    rotor_metal = post
    rotor_glass = None

    for i in range(3):
        angle = i * 120
        rotor_metal = rotor_metal.union(wing_frame.rotate((0, 0, 0), (0, 0, 1), angle))
        rotated_glass = wing_glass_panel.rotate((0, 0, 0), (0, 0, 1), angle)
        if rotor_glass is None:
            rotor_glass = rotated_glass
        else:
            rotor_glass = rotor_glass.union(rotated_glass)

    # Assembly
    drum = model.part("drum")
    drum.visual(mesh_from_cadquery(drum_metal, "drum_metal"), material=mirror_stainless)
    drum.visual(mesh_from_cadquery(drum_glass, "drum_glass"), material=glass_mat)

    rotor = model.part("rotor")
    rotor.visual(mesh_from_cadquery(rotor_metal, "rotor_metal"), material=mirror_stainless)
    rotor.visual(mesh_from_cadquery(rotor_glass, "rotor_glass"), material=glass_mat)

    model.articulation(
        "rotor_joint",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    rotor_joint = object_model.get_articulation("rotor_joint")
    
    ctx.allow_overlap(
        drum, rotor,
        reason="The central post is embedded into the top and bottom pivot mounts."
    )
    
    # Check that the rotor is contained within the drum
    ctx.expect_within(
        rotor, drum,
        axes="xy",
        margin=0.01,
        name="Rotor wings stay within the drum footprint"
    )
    
    # Pose test: Rotor rotates freely without breaking containment
    with ctx.pose({rotor_joint: 1.0}):
        ctx.expect_within(
            rotor, drum,
            axes="xy",
            margin=0.01,
            name="Rotor wings stay within the drum footprint when rotated"
        )

    return ctx.report()

object_model = build_object_model()
