from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_MAT = "dark_green_powder_coat"
HARDWARE_MAT = "blackened_hardware"
STONE_MAT = "weathered_stone"


def _box(part, name, size, xyz, material=FRAME_MAT):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name, radius, length, xyz, material=HARDWARE_MAT):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)


def _add_leaf(leaf, *, side: int) -> None:
    """Build one square-tube garden-gate leaf in the leaf hinge frame.

    side=+1 builds the leaf that runs from its hinge toward +X.
    side=-1 builds the mirrored leaf that runs toward -X.
    """

    y_panel = 0.070
    tube_y = 0.050
    width = 1.550
    bottom_z = 0.080
    top_z = 1.420
    center_z = (bottom_z + top_z) * 0.5
    height = top_z - bottom_z

    # Perimeter square-tube frame.  Rails and stiles intentionally overlap at
    # their mitred-looking corners so the authored leaf is one connected object.
    _box(leaf, "outer_stile", (0.090, tube_y, height), (side * 0.045, y_panel, center_z))
    _box(leaf, "inner_stile", (0.090, tube_y, height), (side * width, y_panel, center_z))
    _box(leaf, "top_rail", (width, tube_y, 0.070), (side * (width * 0.5), y_panel, top_z - 0.035))
    _box(leaf, "bottom_rail", (width, tube_y, 0.070), (side * (width * 0.5), y_panel, bottom_z + 0.035))
    _box(leaf, "mid_rail", (width - 0.070, tube_y * 0.80, 0.045), (side * (width * 0.5), y_panel, 0.760))

    # Sparse vertical infill bars between the perimeter rails.
    for i, x in enumerate((0.34, 0.62, 0.90, 1.18)):
        _box(leaf, f"infill_{i}", (0.034, 0.034, 1.210), (side * x, y_panel, 0.750))

    # Exposed moving hinge knuckles and flat straps on the front face.
    for i, z in enumerate((0.560, 1.040)):
        _cylinder(leaf, f"hinge_knuckle_{i}", 0.026, 0.150, (0.0, 0.0, z))
        _box(leaf, f"hinge_strap_{i}", (0.185, 0.032, 0.050), (side * 0.075, 0.036, z))

    # Center latch furniture.  The meeting stiles are distinct, leaving a real
    # gap between the two leaves at q=0 while still reading as a latch area.
    _box(leaf, "latch_plate", (0.120, 0.016, 0.170), (side * (width - 0.022), 0.039, 0.930), HARDWARE_MAT)
    if side < 0:
        _box(leaf, "latch_handle", (0.210, 0.020, 0.040), (side * (width - 0.120), 0.022, 0.930), HARDWARE_MAT)
        _box(leaf, "latch_boss", (0.046, 0.034, 0.070), (side * (width - 0.160), 0.026, 0.930), HARDWARE_MAT)
    else:
        _box(leaf, "latch_keeper", (0.080, 0.030, 0.085), (side * (width - 0.032), 0.027, 0.930), HARDWARE_MAT)


def _add_drop_bolt_guides(leaf) -> None:
    """Add two front-mounted rectangular guide tubes to the active leaf."""

    rod_x = -1.490
    rod_y = 0.000
    outer_x = 0.086
    outer_y = 0.066
    wall = 0.012
    guide_h = 0.150

    for label, z in (("lower", 0.300), ("upper", 0.790)):
        # Four connected walls create an actual clear central channel.
        _box(
            leaf,
            f"{label}_guide_left_wall",
            (wall, outer_y, guide_h),
            (rod_x - outer_x * 0.5 + wall * 0.5, rod_y, z),
            HARDWARE_MAT,
        )
        _box(
            leaf,
            f"{label}_guide_right_wall",
            (wall, outer_y, guide_h),
            (rod_x + outer_x * 0.5 - wall * 0.5, rod_y, z),
            HARDWARE_MAT,
        )
        _box(
            leaf,
            f"{label}_guide_front_wall",
            (outer_x, wall, guide_h),
            (rod_x, rod_y - outer_y * 0.5 + wall * 0.5, z),
            HARDWARE_MAT,
        )
        _box(
            leaf,
            f"{label}_guide_back_wall",
            (outer_x, wall, guide_h),
            (rod_x, rod_y + outer_y * 0.5 - wall * 0.5, z),
            HARDWARE_MAT,
        )
        # A small welded tab reaches back to the center stile so the tube is not
        # visually floating off the face of the gate.
        _box(leaf, f"{label}_guide_tab", (0.094, 0.022, 0.070), (rod_x, 0.039, z), HARDWARE_MAT)

    _box(leaf, "bolt_slot_plate", (0.050, 0.026, 0.320), (rod_x - 0.060, 0.039, 0.610), HARDWARE_MAT)
    _box(leaf, "bolt_handle_stop", (0.150, 0.022, 0.035), (rod_x - 0.060, 0.025, 0.610), HARDWARE_MAT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_garden_gate")
    model.material(FRAME_MAT, rgba=(0.02, 0.20, 0.10, 1.0))
    model.material(HARDWARE_MAT, rgba=(0.01, 0.012, 0.012, 1.0))
    model.material(STONE_MAT, rgba=(0.42, 0.38, 0.33, 1.0))

    frame = model.part("posts")
    # A continuous stone/soil footing ties the side posts and center receiver
    # together so the stationary assembly is one supported root part.
    _box(frame, "footing", (3.850, 0.320, 0.080), (0.0, 0.120, -0.040), STONE_MAT)
    for side, label in ((-1, "left"), (1, "right")):
        x = side * 1.720
        _box(frame, f"{label}_post", (0.120, 0.120, 1.560), (x, 0.0, 0.750), FRAME_MAT)
        _box(frame, f"{label}_post_cap", (0.160, 0.160, 0.055), (x, 0.0, 1.555), HARDWARE_MAT)

        hinge_x = side * 1.620
        # Stationary hinge leaves/knuckles on the posts.  They are offset on the
        # front face, matching the moving knuckles' hinge line.
        for i, z in enumerate((0.355, 1.235)):
            _box(frame, f"{label}_hinge_plate_{i}", (0.145, 0.030, 0.075), (side * 1.665, -0.057, z), HARDWARE_MAT)
            _cylinder(frame, f"{label}_fixed_knuckle_{i}", 0.026, 0.150, (hinge_x, -0.070, z), HARDWARE_MAT)
        _cylinder(frame, f"{label}_hinge_pin", 0.012, 0.960, (hinge_x, -0.070, 0.800), HARDWARE_MAT)

    # Ground receiver: a short rectangular socket with a real central hole for
    # the drop bolt, connected back to the footing by a welded tail.
    receiver_x = 0.130
    receiver_y = -0.070
    receiver_z = -0.048
    _box(frame, "receiver_left_wall", (0.014, 0.112, 0.096), (receiver_x - 0.056, receiver_y, receiver_z), HARDWARE_MAT)
    _box(frame, "receiver_right_wall", (0.014, 0.112, 0.096), (receiver_x + 0.056, receiver_y, receiver_z), HARDWARE_MAT)
    _box(frame, "receiver_front_wall", (0.126, 0.014, 0.096), (receiver_x, receiver_y - 0.056, receiver_z), HARDWARE_MAT)
    _box(frame, "receiver_back_wall", (0.126, 0.014, 0.096), (receiver_x, receiver_y + 0.056, receiver_z), HARDWARE_MAT)
    _box(frame, "receiver_tail", (0.070, 0.094, 0.018), (receiver_x, 0.025, -0.008), HARDWARE_MAT)

    left_leaf = model.part("leaf_0")
    _add_leaf(left_leaf, side=1)
    right_leaf = model.part("leaf_1")
    _add_leaf(right_leaf, side=-1)
    _add_drop_bolt_guides(right_leaf)

    drop_bolt = model.part("drop_bolt")
    _cylinder(drop_bolt, "bolt_rod", 0.015, 1.050, (0.0, 0.0, 0.450), HARDWARE_MAT)
    _cylinder(drop_bolt, "bolt_tip", 0.020, 0.060, (0.0, 0.0, -0.060), HARDWARE_MAT)
    _box(drop_bolt, "grip_stem", (0.032, 0.058, 0.030), (-0.010, -0.026, 0.620), HARDWARE_MAT)
    _box(drop_bolt, "hand_grip", (0.170, 0.030, 0.036), (-0.065, -0.058, 0.620), HARDWARE_MAT)

    left_hinge = model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_leaf,
        origin=Origin(xyz=(-1.620, -0.070, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=1.65),
    )
    right_hinge = model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_leaf,
        origin=Origin(xyz=(1.620, -0.070, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=1.65),
    )
    model.articulation(
        "bolt_slide",
        ArticulationType.PRISMATIC,
        parent=right_leaf,
        child=drop_bolt,
        origin=Origin(xyz=(-1.490, 0.000, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=0.320),
    )
    # Avoid lint-like unused local complaints in stricter runners while keeping
    # semantic names next to their construction.
    _ = (left_hinge, right_hinge)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_leaf = object_model.get_part("leaf_0")
    right_leaf = object_model.get_part("leaf_1")
    posts = object_model.get_part("posts")
    drop_bolt = object_model.get_part("drop_bolt")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    bolt_slide = object_model.get_articulation("bolt_slide")

    for leaf, pin_name in ((left_leaf, "left_hinge_pin"), (right_leaf, "right_hinge_pin")):
        for i in (0, 1):
            knuckle_name = f"hinge_knuckle_{i}"
            ctx.allow_overlap(
                leaf,
                posts,
                elem_a=knuckle_name,
                elem_b=pin_name,
                reason="The post-mounted hinge pin is intentionally captured inside the leaf hinge barrel.",
            )
            ctx.expect_within(
                posts,
                leaf,
                axes="xy",
                inner_elem=pin_name,
                outer_elem=knuckle_name,
                name=f"{leaf.name} hinge pin is centered in {knuckle_name}",
            )
            ctx.expect_overlap(
                leaf,
                posts,
                axes="z",
                min_overlap=0.120,
                elem_a=knuckle_name,
                elem_b=pin_name,
                name=f"{leaf.name} hinge barrel wraps pin vertically {i}",
            )

    # Closed leaves meet at the center with a real narrow clearance rather than
    # overlapping; their rails still line up vertically like a paired gate.
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        min_gap=0.018,
        max_gap=0.085,
        positive_elem="inner_stile",
        negative_elem="inner_stile",
        name="meeting stiles have a narrow center gap",
    )
    ctx.expect_overlap(
        left_leaf,
        right_leaf,
        axes="z",
        min_overlap=1.20,
        elem_a="inner_stile",
        elem_b="inner_stile",
        name="meeting stiles align in height",
    )

    # The closed drop bolt sits in the ground receiver and through both guide
    # tubes, but it is clear of the tube walls because the guides are built as
    # open rectangular sleeves.
    ctx.expect_overlap(
        drop_bolt,
        posts,
        axes="z",
        min_overlap=0.040,
        elem_a="bolt_rod",
        elem_b="receiver_back_wall",
        name="lowered drop bolt reaches receiver depth",
    )
    ctx.expect_overlap(
        drop_bolt,
        right_leaf,
        axes="z",
        min_overlap=0.120,
        elem_a="bolt_rod",
        elem_b="lower_guide_back_wall",
        name="drop bolt passes through lower guide height",
    )
    ctx.expect_overlap(
        drop_bolt,
        right_leaf,
        axes="z",
        min_overlap=0.120,
        elem_a="bolt_rod",
        elem_b="upper_guide_back_wall",
        name="drop bolt passes through upper guide height",
    )

    def _elem_center_z(part, elem):
        lo, hi = ctx.part_element_world_aabb(part, elem=elem)
        return (lo[2] + hi[2]) * 0.5

    def _elem_center_y(part, elem):
        lo, hi = ctx.part_element_world_aabb(part, elem=elem)
        return (lo[1] + hi[1]) * 0.5

    left_closed_y = _elem_center_y(left_leaf, "inner_stile")
    right_closed_y = _elem_center_y(right_leaf, "inner_stile")
    bolt_low_z = _elem_center_z(drop_bolt, "bolt_tip")

    with ctx.pose({left_hinge: 1.20, right_hinge: 1.20, bolt_slide: 0.320}):
        left_open_y = _elem_center_y(left_leaf, "inner_stile")
        right_open_y = _elem_center_y(right_leaf, "inner_stile")
        bolt_high_z = _elem_center_z(drop_bolt, "bolt_tip")

    ctx.check(
        "both leaves swing outward on vertical hinges",
        left_open_y > left_closed_y + 0.45 and right_open_y > right_closed_y + 0.45,
        details=f"closed_y=({left_closed_y:.3f}, {right_closed_y:.3f}), open_y=({left_open_y:.3f}, {right_open_y:.3f})",
    )
    ctx.check(
        "drop bolt retracts upward in guide tubes",
        bolt_high_z > bolt_low_z + 0.280,
        details=f"lowered_tip_z={bolt_low_z:.3f}, raised_tip_z={bolt_high_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
