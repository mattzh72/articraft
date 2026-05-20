from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_section(width_x: float, depth_y: float, z: float, radius: float):
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(width_x, depth_y, radius, corner_segments=8)
    ]


def _limb_shell_mesh(name: str, sections: tuple[tuple[float, float, float, float], ...]):
    """Low-cost tapered molded/extruded limb shell: rounded rectangles lofted along Z."""
    profiles = tuple(_rounded_section(width, depth, z, radius) for width, depth, z, radius in sections)
    return mesh_from_geometry(LoftGeometry(profiles, cap=True, closed=True), name)


def _bolt_head(
    part,
    *,
    x: float,
    y: float,
    z: float,
    material,
    name: str,
    radius: float = 0.007,
):
    part.visual(
        Cylinder(radius=radius, length=0.007),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_robotic_leg")

    shell = model.material("glass_filled_nylon_shell", rgba=(0.22, 0.24, 0.25, 1.0))
    dark = model.material("black_actuator_bay", rgba=(0.03, 0.035, 0.04, 1.0))
    metal = model.material("zinc_plated_fasteners", rgba=(0.70, 0.68, 0.61, 1.0))
    rubber = model.material("molded_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    accent = model.material("snap_tab_markers", rgba=(0.95, 0.62, 0.16, 1.0))

    # Root hip bracket: one stamped/molded pelvis-side carrier with integrated cheeks.
    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        Box((0.22, 0.31, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=shell,
        name="mounting_flange",
    )
    hip_mount.visual(
        Box((0.18, 0.26, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=shell,
        name="pelvis_bridge",
    )
    for idx, y in enumerate((-0.075, 0.075)):
        hip_mount.visual(
            Box((0.105, 0.030, 0.200)),
            origin=Origin(xyz=(0.0, y, -0.035)),
            material=shell,
            name=f"hip_cheek_{idx}",
        )
    hip_mount.visual(
        Cylinder(radius=0.024, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hip_axle",
    )
    for idx, y in enumerate((-0.103, 0.103)):
        hip_mount.visual(
            Cylinder(radius=0.068, length=0.032),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"hip_motor_cap_{idx}",
        )

    # Upper leg: one large molded shell with a molded-in actuator bay and a knee fork.
    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.044, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shell,
        name="hip_hub",
    )
    thigh.visual(
        Box((0.070, 0.064, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=shell,
        name="hip_neck",
    )
    thigh.visual(
        _limb_shell_mesh(
            "thigh_shell",
            (
                (0.118, 0.078, -0.080, 0.014),
                (0.108, 0.071, -0.240, 0.012),
                (0.096, 0.062, -0.390, 0.010),
            ),
        ),
        material=shell,
        name="thigh_shell",
    )
    thigh.visual(
        Box((0.010, 0.058, 0.205)),
        origin=Origin(xyz=(0.059, 0.0, -0.245)),
        material=dark,
        name="thigh_bay_cover",
    )
    for idx, (y, z) in enumerate(((-0.021, -0.165), (0.021, -0.165), (-0.021, -0.325), (0.021, -0.325))):
        _bolt_head(thigh, x=0.066, y=y, z=z, material=metal, name=f"thigh_cover_bolt_{idx}")
    for idx, z in enumerate((-0.195, -0.295)):
        thigh.visual(
            Box((0.014, 0.070, 0.014)),
            origin=Origin(xyz=(0.064, 0.0, z)),
            material=accent,
            name=f"thigh_snap_tab_{idx}",
        )
    thigh.visual(
        Box((0.095, 0.170, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.395)),
        material=shell,
        name="knee_bridge",
    )
    for idx, y in enumerate((-0.072, 0.072)):
        thigh.visual(
            Box((0.105, 0.026, 0.150)),
            origin=Origin(xyz=(0.0, y, -0.475)),
            material=shell,
            name=f"knee_fork_plate_{idx}",
        )
    thigh.visual(
        Cylinder(radius=0.023, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.480), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="knee_axle",
    )
    for idx, y in enumerate((-0.091, 0.091)):
        thigh.visual(
            Cylinder(radius=0.039, length=0.014),
            origin=Origin(xyz=(0.0, y, -0.480), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"knee_clamp_cap_{idx}",
        )

    # Lower leg: a slimmer extrusion-like shell with an ankle fork molded as the same part.
    shin = model.part("shin")
    shin.visual(
        Cylinder(radius=0.041, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shell,
        name="knee_hub",
    )
    shin.visual(
        Box((0.062, 0.058, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=shell,
        name="knee_neck",
    )
    shin.visual(
        _limb_shell_mesh(
            "shin_shell",
            (
                (0.096, 0.066, -0.090, 0.011),
                (0.088, 0.060, -0.230, 0.010),
                (0.080, 0.052, -0.360, 0.009),
            ),
        ),
        material=shell,
        name="shin_shell",
    )
    shin.visual(
        Box((0.009, 0.050, 0.170)),
        origin=Origin(xyz=(0.049, 0.0, -0.230)),
        material=dark,
        name="shin_bay_cover",
    )
    for idx, (y, z) in enumerate(((-0.018, -0.165), (0.018, -0.165), (-0.018, -0.295), (0.018, -0.295))):
        _bolt_head(shin, x=0.055, y=y, z=z, material=metal, name=f"shin_cover_bolt_{idx}", radius=0.006)
    shin.visual(
        Box((0.082, 0.148, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
        material=shell,
        name="ankle_bridge",
    )
    for idx, y in enumerate((-0.064, 0.064)):
        shin.visual(
            Box((0.090, 0.024, 0.130)),
            origin=Origin(xyz=(0.0, y, -0.425)),
            material=shell,
            name=f"ankle_fork_plate_{idx}",
        )
    shin.visual(
        Cylinder(radius=0.020, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.430), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="ankle_axle",
    )
    for idx, y in enumerate((-0.082, 0.082)):
        shin.visual(
            Cylinder(radius=0.033, length=0.012),
            origin=Origin(xyz=(0.0, y, -0.430), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"ankle_clamp_cap_{idx}",
        )

    # Foot: a simple two-shot sole and forward web mounted to the ankle hub.
    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.038, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shell,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.045, 0.065, 0.110)),
        origin=Origin(xyz=(0.050, 0.0, -0.070)),
        material=shell,
        name="ankle_web",
    )
    foot.visual(
        Box((0.280, 0.120, 0.035)),
        origin=Origin(xyz=(0.075, 0.0, -0.125)),
        material=shell,
        name="sole_plate",
    )
    foot.visual(
        Box((0.125, 0.105, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, -0.149)),
        material=rubber,
        name="toe_pad",
    )
    foot.visual(
        Box((0.090, 0.105, 0.018)),
        origin=Origin(xyz=(-0.045, 0.0, -0.149)),
        material=rubber,
        name="heel_pad",
    )
    for idx, x in enumerate((0.115, 0.160, 0.205)):
        foot.visual(
            Box((0.012, 0.100, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.160)),
            material=rubber,
            name=f"toe_tread_{idx}",
        )
    for idx, x in enumerate((-0.070, -0.025)):
        foot.visual(
            Box((0.012, 0.100, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.160)),
            material=rubber,
            name=f"heel_tread_{idx}",
        )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.5, lower=-0.75, upper=1.15),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shin,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=2.8, lower=0.0, upper=2.05),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shin,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=3.0, lower=-0.55, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip_mount = object_model.get_part("hip_mount")
    thigh = object_model.get_part("thigh")
    shin = object_model.get_part("shin")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.allow_overlap(
        hip_mount,
        thigh,
        elem_a="hip_axle",
        elem_b="hip_hub",
        reason="The thigh hub is intentionally captured around the hip axle/bushing proxy.",
    )
    ctx.allow_overlap(
        thigh,
        shin,
        elem_a="knee_axle",
        elem_b="knee_hub",
        reason="The shin hub is intentionally captured around the knee axle/bushing proxy.",
    )
    ctx.allow_overlap(
        shin,
        foot,
        elem_a="ankle_axle",
        elem_b="ankle_hub",
        reason="The foot hub is intentionally captured around the ankle axle/bushing proxy.",
    )

    ctx.check(
        "serial hip knee ankle chain",
        hip.parent == "hip_mount"
        and hip.child == "thigh"
        and knee.parent == "thigh"
        and knee.child == "shin"
        and ankle.parent == "shin"
        and ankle.child == "foot",
        details=f"chain={(hip.parent, hip.child, knee.parent, knee.child, ankle.parent, ankle.child)}",
    )
    ctx.check(
        "plausible pitch joint limits",
        hip.motion_limits.lower < 0.0
        and hip.motion_limits.upper > 0.8
        and knee.motion_limits.lower == 0.0
        and knee.motion_limits.upper > 1.6
        and ankle.motion_limits.lower < 0.0
        and ankle.motion_limits.upper > 0.4,
        details="expected hip bidirectional, knee folding, and ankle bidirectional pitch ranges",
    )

    for parent, child, axle, hub, label in (
        (hip_mount, thigh, "hip_axle", "hip_hub", "hip captured bearing"),
        (thigh, shin, "knee_axle", "knee_hub", "knee captured bearing"),
        (shin, foot, "ankle_axle", "ankle_hub", "ankle captured bearing"),
    ):
        ctx.expect_overlap(
            parent,
            child,
            elem_a=axle,
            elem_b=hub,
            axes="xyz",
            min_overlap=0.025,
            name=f"{label} has real retained insertion",
        )
        ctx.expect_within(
            parent,
            child,
            inner_elem=axle,
            outer_elem=hub,
            axes="xz",
            margin=0.004,
            name=f"{label} axle remains inside hub bore envelope",
        )

    rest_knee = ctx.part_world_position(shin)
    with ctx.pose({hip: 0.55}):
        flexed_knee = ctx.part_world_position(shin)
    ctx.check(
        "hip flexion moves knee forward",
        rest_knee is not None and flexed_knee is not None and flexed_knee[0] > rest_knee[0] + 0.20,
        details=f"rest={rest_knee}, flexed={flexed_knee}",
    )

    rest_ankle = ctx.part_world_position(foot)
    with ctx.pose({knee: 0.85}):
        folded_ankle = ctx.part_world_position(foot)
    ctx.check(
        "knee pitch folds the lower leg",
        rest_ankle is not None
        and folded_ankle is not None
        and folded_ankle[0] < rest_ankle[0] - 0.25,
        details=f"rest={rest_ankle}, folded={folded_ankle}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_toe = _aabb_center(ctx.part_element_world_aabb(foot, elem="toe_pad"))
    with ctx.pose({ankle: 0.35}):
        lifted_toe = _aabb_center(ctx.part_element_world_aabb(foot, elem="toe_pad"))
    ctx.check(
        "ankle pitch lifts toe pad",
        rest_toe is not None and lifted_toe is not None and lifted_toe[2] > rest_toe[2] + 0.045,
        details=f"rest={rest_toe}, lifted={lifted_toe}",
    )

    return ctx.report()


object_model = build_object_model()
