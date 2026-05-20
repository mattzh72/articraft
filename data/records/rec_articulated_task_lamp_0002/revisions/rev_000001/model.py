from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _segment_pose(dx: float, dz: float, y: float, start: float, end: float) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    center = ((start + end) * 0.5)
    xyz = (
        math.cos(angle) * center,
        y,
        math.sin(angle) * center,
    )
    size = (end - start, 0.0, 0.0)
    return xyz, (0.0, -angle, 0.0), size


def _helical_spring_geometry(
    dx: float,
    dz: float,
    axial_start: float,
    axial_end: float,
    coil_radius: float,
    turns: int,
    wire_radius: float,
):
    length = math.hypot(dx, dz)
    ux = dx / length
    uz = dz / length
    normal_a = (0.0, 1.0, 0.0)
    normal_b = (-uz, 0.0, ux)
    steps = turns * 20
    points: list[tuple[float, float, float]] = []
    for i in range(steps + 1):
        t = i / steps
        axial = axial_start + (axial_end - axial_start) * t
        phase = turns * 2.0 * math.pi * t
        radial_y = coil_radius * math.cos(phase)
        radial_b = coil_radius * math.sin(phase)
        points.append(
            (
                ux * axial + normal_a[0] * radial_y + normal_b[0] * radial_b,
                normal_a[1] * radial_y + normal_b[1] * radial_b,
                uz * axial + normal_a[2] * radial_y + normal_b[2] * radial_b,
            )
        )
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=3,
        radial_segments=14,
        cap_ends=True,
    )


def _make_shade_mesh():
    shade = LatheGeometry.from_shell_profiles(
        [
            (0.014, 0.000),
            (0.020, 0.010),
            (0.030, 0.026),
            (0.042, 0.050),
            (0.051, 0.078),
        ],
        [
            (0.000, 0.004),
            (0.010, 0.006),
            (0.017, 0.015),
            (0.026, 0.031),
            (0.044, 0.072),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    shade.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shade, ASSETS.mesh_path("architect_lamp_shade.obj"))


def _add_arm_segment(
    part,
    prefix: str,
    dx: float,
    dz: float,
    arm_material,
    hardware_material,
    spring_material,
):
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    rail_y = 0.010
    rail_width = 0.006
    rail_thickness = 0.004
    rail_start = 0.022
    rail_end = length
    spine_end = length - 0.030
    spring_mesh = mesh_from_geometry(
        _helical_spring_geometry(
            dx,
            dz,
            axial_start=0.055,
            axial_end=length - 0.050,
            coil_radius=0.0035,
            turns=8,
            wire_radius=0.0011,
        ),
        ASSETS.mesh_path(f"{prefix}_spring.obj"),
    )

    part.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name=f"{prefix}_hub",
    )

    part.visual(
        Box((spine_end, 0.008, 0.004)),
        origin=Origin(
            xyz=(math.cos(angle) * (spine_end * 0.5), 0.0, math.sin(angle) * (spine_end * 0.5)),
            rpy=(0.0, -angle, 0.0),
        ),
        material=arm_material,
        name=f"{prefix}_spine",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        y = sign * rail_y
        part.visual(
            Box((rail_end - rail_start, rail_width, rail_thickness)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * ((rail_start + rail_end) * 0.5),
                    y,
                    math.sin(angle) * ((rail_start + rail_end) * 0.5),
                ),
                rpy=(0.0, -angle, 0.0),
            ),
            material=arm_material,
            name=f"{prefix}_rail_{side}",
        )
        for label, axial in (("rear", 0.050), ("front", length - 0.050)):
            part.visual(
                Box((0.014, 0.004, 0.004)),
                origin=Origin(
                    xyz=(math.cos(angle) * axial, sign * 0.015, math.sin(angle) * axial),
                    rpy=(0.0, -angle, 0.0),
                ),
                material=arm_material,
                name=f"{prefix}_{label}_tab_{side}",
            )
        part.visual(
            spring_mesh,
            origin=Origin(xyz=(0.0, sign * 0.015, 0.0)),
            material=spring_material,
            name=f"{prefix}_spring_{side}",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(dx, sign * 0.013, dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_material,
            name=f"{prefix}_yoke_{side}",
        )

    for label, axial in (("rear", 0.045), ("front", length - 0.045)):
        part.visual(
            Box((0.010, 0.024, 0.006)),
            origin=Origin(
                xyz=(math.cos(angle) * axial, 0.0, math.sin(angle) * axial),
                rpy=(0.0, -angle, 0.0),
            ),
            material=arm_material,
            name=f"{prefix}_crossbar_{label}",
        )

    return length, angle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_desk_lamp", assets=ASSETS)

    base_material = model.material("painted_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    arm_material = model.material("lamp_enamel", rgba=(0.80, 0.82, 0.76, 1.0))
    hardware_material = model.material("brushed_steel", rgba=(0.60, 0.63, 0.66, 1.0))
    spring_material = model.material("spring_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    bulb_material = model.material("warm_bulb", rgba=(0.98, 0.92, 0.72, 0.92))

    lower_dx = 0.205
    lower_dz = 0.152
    upper_dx = 0.185
    upper_dz = 0.128
    pivot_z = 0.078
    shade_mesh = _make_shade_mesh()

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_material,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=base_material,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=hardware_material,
        name="pedestal",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(0.0, sign * 0.013, pivot_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_material,
            name=f"pivot_washer_{side}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.028),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    lower_arm = model.part("lower_arm")
    _add_arm_segment(lower_arm, "lower", lower_dx, lower_dz, arm_material, hardware_material, spring_material)
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.215, 0.045, 0.170)),
        mass=0.35,
        origin=Origin(xyz=(0.105, 0.0, 0.078)),
    )

    upper_arm = model.part("upper_arm")
    _add_arm_segment(upper_arm, "upper", upper_dx, upper_dz, arm_material, hardware_material, spring_material)
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.195, 0.045, 0.145)),
        mass=0.28,
        origin=Origin(xyz=(0.095, 0.0, 0.066)),
    )

    head = model.part("head")
    head_pitch = -0.34
    head.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="head_hub",
    )
    head.visual(
        Box((0.030, 0.012, 0.012)),
        origin=Origin(
            xyz=(0.018, 0.0, -0.006),
            rpy=(0.0, head_pitch, 0.0),
        ),
        material=arm_material,
        name="head_neck",
    )
    head.visual(
        shade_mesh,
        origin=Origin(
            xyz=(0.030, 0.0, -0.006),
            rpy=(0.0, head_pitch, 0.0),
        ),
        material=arm_material,
        name="shade_shell",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(0.034, 0.0, -0.008),
            rpy=(0.0, head_pitch, 0.0),
        ),
        material=hardware_material,
        name="socket",
    )
    head.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.078, 0.0, -0.022)),
        material=bulb_material,
        name="bulb",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.140, 0.060, 0.085)),
        mass=0.32,
        origin=Origin(xyz=(0.070, 0.0, -0.020)),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.35, upper=1.05),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(lower_dx, 0.0, lower_dz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.8, lower=-1.10, upper=0.85),
    )
    model.articulation(
        "upper_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(upper_dx, 0.0, upper_dz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.90, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_head = object_model.get_articulation("upper_to_head")
    base_disk = base.get_visual("base_disk")
    base_left = base.get_visual("pivot_washer_left")
    base_right = base.get_visual("pivot_washer_right")
    lower_hub = lower_arm.get_visual("lower_hub")
    lower_left = lower_arm.get_visual("lower_yoke_left")
    lower_right = lower_arm.get_visual("lower_yoke_right")
    upper_hub = upper_arm.get_visual("upper_hub")
    upper_left = upper_arm.get_visual("upper_yoke_left")
    upper_right = upper_arm.get_visual("upper_yoke_right")
    head_hub = head.get_visual("head_hub")
    shade_shell = head.get_visual("shade_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(lower_arm, base, elem_a=lower_hub, elem_b=base_left)
    ctx.expect_contact(lower_arm, base, elem_a=lower_hub, elem_b=base_right)
    ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_hub, elem_b=lower_left)
    ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_hub, elem_b=lower_right)
    ctx.expect_contact(head, upper_arm, elem_a=head_hub, elem_b=upper_left)
    ctx.expect_contact(head, upper_arm, elem_a=head_hub, elem_b=upper_right)
    ctx.expect_origin_distance(lower_arm, base, axes="xy", max_dist=0.002)
    ctx.expect_within(
        lower_arm,
        base,
        axes="xy",
        inner_elem=lower_hub,
        outer_elem=base_disk,
        name="lower_hinge_sits_within_weighted_base_footprint",
    )
    ctx.expect_gap(
        head,
        head,
        axis="x",
        min_gap=0.010,
        positive_elem=shade_shell,
        negative_elem=head_hub,
        name="conical_shade_projects_forward_of_head_pivot",
    )
    ctx.expect_overlap(
        head,
        head,
        axes="yz",
        min_overlap=0.020,
        elem_a=head.get_visual("bulb"),
        elem_b=shade_shell,
        name="bulb_is_visually_sheltered_by_shade",
    )
    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.060,
        positive_elem=shade_shell,
        negative_elem=base_disk,
        name="shade_clears_weighted_base_at_rest",
    )
    with ctx.pose({base_to_lower: 0.65, lower_to_upper: -0.55, upper_to_head: -0.15}):
        ctx.expect_contact(head, upper_arm, elem_a=head_hub, elem_b=upper_left)
        ctx.expect_contact(head, upper_arm, elem_a=head_hub, elem_b=upper_right)
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.020,
            positive_elem=shade_shell,
            negative_elem=base_disk,
            name="task_pose_shade_stays_above_base",
        )
    with ctx.pose({base_to_lower: -0.20, lower_to_upper: -0.40, upper_to_head: 0.00}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.300,
            positive_elem=shade_shell,
            negative_elem=base_disk,
            name="raised_pose_head_lifts_high_over_base",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
