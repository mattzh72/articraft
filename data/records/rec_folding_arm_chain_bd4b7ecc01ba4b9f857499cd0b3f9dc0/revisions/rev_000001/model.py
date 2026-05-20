from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _y_cylinder(part, *, radius, length, xyz, material, name):
    """Add a cylinder whose barrel axis is the mechanism's Y hinge axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _z_cylinder(part, *, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _angled_web(part, *, start, end, y, width, height, material, name):
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    length = sqrt(dx * dx + dz * dz)
    pitch = -atan2(dz, dx)
    part.visual(
        Box((length, width, height)),
        origin=Origin(
            xyz=((start[0] + end[0]) / 2.0, y, (start[1] + end[1]) / 2.0),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _bore_caps(part, *, center_xz, y, width, radius, material, prefix):
    # Shallow dark disks on both faces make each solid boss read as a bored eye.
    x, z = center_xz
    cap_len = 0.004
    _y_cylinder(
        part,
        radius=radius,
        length=cap_len,
        xyz=(x, y + width / 2.0 + 0.001, z),
        material=material,
        name=f"{prefix}_bore_outer",
    )
    _y_cylinder(
        part,
        radius=radius,
        length=cap_len,
        xyz=(x, y - width / 2.0 - 0.001, z),
        material=material,
        name=f"{prefix}_bore_inner",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_folding_arm_chain")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.18, 0.26, 0.34, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.58, 0.52, 0.43, 1.0))
    bronze = model.material("bronze_bushing", rgba=(0.80, 0.55, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    pin_metal = model.material("polished_pin", rgba=(0.72, 0.72, 0.70, 1.0))

    base = model.part("base_foot")
    base.visual(
        Box((0.56, 0.28, 0.035)),
        origin=Origin(xyz=(0.13, 0.0, -0.1825)),
        material=dark_steel,
        name="ground_sole",
    )
    base.visual(
        Box((0.25, 0.16, 0.075)),
        origin=Origin(xyz=(-0.025, 0.0, -0.128)),
        material=dark_steel,
        name="raised_plinth",
    )
    base.visual(
        Box((0.12, 0.024, 0.190)),
        origin=Origin(xyz=(-0.010, -0.084, -0.075)),
        material=dark_steel,
        name="clevis_cheek_0",
    )
    base.visual(
        Box((0.12, 0.024, 0.190)),
        origin=Origin(xyz=(-0.010, 0.026, -0.075)),
        material=dark_steel,
        name="clevis_cheek_1",
    )
    _y_cylinder(
        base,
        radius=0.064,
        length=0.024,
        xyz=(0.0, -0.084, 0.0),
        material=dark_steel,
        name="clevis_boss_0",
    )
    _y_cylinder(
        base,
        radius=0.064,
        length=0.024,
        xyz=(0.0, 0.026, 0.0),
        material=dark_steel,
        name="clevis_boss_1",
    )
    _y_cylinder(
        base,
        radius=0.020,
        length=0.130,
        xyz=(0.0, -0.030, 0.0),
        material=pin_metal,
        name="root_pivot_pin",
    )
    for i, (x, y) in enumerate(
        ((-0.10, -0.095), (-0.10, 0.095), (0.32, -0.095), (0.32, 0.095))
    ):
        _z_cylinder(
            base,
            radius=0.017,
            length=0.006,
            xyz=(x, y, -0.165),
            material=rubber,
            name=f"sole_bolt_{i}",
        )

    root = model.part("root_rocker")
    root_y = -0.030
    root_w = 0.052
    root_tip = (0.380, 0.055)
    _angled_web(
        root,
        start=(0.050, 0.007),
        end=root_tip,
        y=root_y,
        width=root_w,
        height=0.040,
        material=blued_steel,
        name="sloped_web",
    )
    _angled_web(
        root,
        start=(0.055, 0.008),
        end=(0.195, 0.028),
        y=root_y,
        width=0.070,
        height=0.060,
        material=blued_steel,
        name="root_shoulder",
    )
    _y_cylinder(
        root,
        radius=0.052,
        length=root_w,
        xyz=(0.0, root_y, 0.0),
        material=blued_steel,
        name="proximal_boss",
    )
    _y_cylinder(
        root,
        radius=0.047,
        length=root_w,
        xyz=(root_tip[0], root_y, root_tip[1]),
        material=blued_steel,
        name="distal_boss",
    )
    _y_cylinder(
        root,
        radius=0.020,
        length=0.019,
        xyz=(root_tip[0], 0.0045, root_tip[1]),
        material=pin_metal,
        name="distal_pin_sleeve",
    )
    _bore_caps(
        root,
        center_xz=(0.0, 0.0),
        y=root_y,
        width=root_w,
        radius=0.021,
        material=rubber,
        prefix="proximal",
    )
    _bore_caps(
        root,
        center_xz=root_tip,
        y=root_y,
        width=root_w,
        radius=0.019,
        material=rubber,
        prefix="distal",
    )

    mid = model.part("middle_link")
    mid_y = 0.036
    mid_w = 0.044
    mid_tip = (0.320, -0.095)
    _angled_web(
        mid,
        start=(0.0, 0.0),
        end=mid_tip,
        y=mid_y,
        width=mid_w,
        height=0.036,
        material=warm_steel,
        name="dropped_web",
    )
    _angled_web(
        mid,
        start=(0.120, -0.035),
        end=(0.320, -0.095),
        y=mid_y,
        width=0.056,
        height=0.028,
        material=warm_steel,
        name="lower_step",
    )
    _y_cylinder(
        mid,
        radius=0.043,
        length=mid_w,
        xyz=(0.0, mid_y, 0.0),
        material=warm_steel,
        name="proximal_boss",
    )
    _y_cylinder(
        mid,
        radius=0.038,
        length=mid_w,
        xyz=(mid_tip[0], mid_y, mid_tip[1]),
        material=warm_steel,
        name="distal_boss",
    )
    _y_cylinder(
        mid,
        radius=0.017,
        length=0.009,
        xyz=(mid_tip[0], 0.0105, mid_tip[1]),
        material=pin_metal,
        name="distal_pin_sleeve",
    )
    _bore_caps(
        mid,
        center_xz=(0.0, 0.0),
        y=mid_y,
        width=mid_w,
        radius=0.018,
        material=rubber,
        prefix="proximal",
    )
    _bore_caps(
        mid,
        center_xz=mid_tip,
        y=mid_y,
        width=mid_w,
        radius=0.016,
        material=rubber,
        prefix="distal",
    )

    tip = model.part("tip_link")
    tip_y = -0.012
    tip_w = 0.035
    tip_end = (0.260, 0.035)
    _angled_web(
        tip,
        start=(0.0, 0.0),
        end=tip_end,
        y=tip_y,
        width=tip_w,
        height=0.030,
        material=bronze,
        name="narrow_web",
    )
    _y_cylinder(
        tip,
        radius=0.034,
        length=tip_w,
        xyz=(0.0, tip_y, 0.0),
        material=bronze,
        name="proximal_boss",
    )
    _y_cylinder(
        tip,
        radius=0.029,
        length=tip_w,
        xyz=(tip_end[0], tip_y, tip_end[1]),
        material=bronze,
        name="output_boss",
    )
    tip.visual(
        Box((0.082, tip_w, 0.025)),
        origin=Origin(xyz=(tip_end[0] + 0.043, tip_y, tip_end[1])),
        material=bronze,
        name="output_tab",
    )
    _bore_caps(
        tip,
        center_xz=(0.0, 0.0),
        y=tip_y,
        width=tip_w,
        radius=0.014,
        material=rubber,
        prefix="proximal",
    )
    _bore_caps(
        tip,
        center_xz=tip_end,
        y=tip_y,
        width=tip_w,
        radius=0.012,
        material=rubber,
        prefix="output",
    )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=base,
        child=root,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-0.75, upper=1.15),
    )
    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=root,
        child=mid,
        origin=Origin(xyz=(root_tip[0], 0.0, root_tip[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.8, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "middle_to_tip",
        ArticulationType.REVOLUTE,
        parent=mid,
        child=tip,
        origin=Origin(xyz=(mid_tip[0], 0.0, mid_tip[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.20, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_foot")
    root = object_model.get_part("root_rocker")
    mid = object_model.get_part("middle_link")
    tip = object_model.get_part("tip_link")
    base_to_root = object_model.get_articulation("base_to_root")
    root_to_middle = object_model.get_articulation("root_to_middle")
    middle_to_tip = object_model.get_articulation("middle_to_tip")

    ctx.allow_overlap(
        base,
        root,
        elem_a="root_pivot_pin",
        elem_b="proximal_boss",
        reason="The grounded hinge pin is intentionally modeled through the solid root boss proxy.",
    )

    revolute = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "serial chain has exactly three revolute joints",
        len(revolute) == 3,
        details=f"revolute joints={[joint.name for joint in revolute]}",
    )
    ctx.check(
        "all hinge axes share the single motion plane",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0) for joint in revolute),
        details=f"axes={[joint.axis for joint in revolute]}",
    )

    ctx.expect_overlap(
        root,
        base,
        axes="xz",
        elem_a="proximal_boss",
        elem_b="clevis_boss_1",
        min_overlap=0.030,
        name="root boss is captured on base hinge line",
    )
    ctx.expect_within(
        base,
        root,
        axes="xz",
        inner_elem="root_pivot_pin",
        outer_elem="proximal_boss",
        margin=0.001,
        name="root pivot pin is centered inside the root boss",
    )
    ctx.expect_overlap(
        base,
        root,
        axes="y",
        elem_a="root_pivot_pin",
        elem_b="proximal_boss",
        min_overlap=0.045,
        name="root pivot pin spans the root boss width",
    )
    ctx.expect_overlap(
        mid,
        root,
        axes="xz",
        elem_a="proximal_boss",
        elem_b="distal_boss",
        min_overlap=0.030,
        name="middle link shares the root distal hinge line",
    )
    ctx.expect_overlap(
        tip,
        mid,
        axes="xz",
        elem_a="proximal_boss",
        elem_b="distal_boss",
        min_overlap=0.025,
        name="tip link shares the middle distal hinge line",
    )
    ctx.expect_gap(
        mid,
        root,
        axis="y",
        positive_elem="proximal_boss",
        negative_elem="distal_boss",
        min_gap=0.006,
        max_gap=0.030,
        name="root to middle hinge has visible offset spacer gap",
    )
    ctx.expect_gap(
        mid,
        tip,
        axis="y",
        positive_elem="distal_boss",
        negative_elem="proximal_boss",
        min_gap=0.004,
        max_gap=0.018,
        name="middle to tip hinge has a smaller offset spacer gap",
    )

    with ctx.pose({base_to_root: 0.65, root_to_middle: -0.85, middle_to_tip: 0.75}):
        positions = [ctx.part_world_position(part) for part in (root, mid, tip)]
        ctx.check(
            "folded pose remains in the XZ motion plane",
            all(pos is not None and abs(pos[1]) < 1e-6 for pos in positions),
            details=f"joint-center positions={positions}",
        )

    return ctx.report()


object_model = build_object_model()
