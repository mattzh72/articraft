from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_height_turnstile")

    painted_steel = model.material("painted_steel", rgba=(0.24, 0.28, 0.31, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.71, 0.74, 0.76, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.13, 0.14, 0.15, 1.0))

    support = model.part("support_post")
    support.visual(
        Cylinder(radius=0.23, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_steel,
        name="floor_flange",
    )
    support.visual(
        Cylinder(radius=0.08, length=2.00),
        origin=Origin(xyz=(0.0, 0.0, 1.03)),
        material=galvanized_steel,
        name="central_post",
    )
    support.visual(
        Cylinder(radius=0.11, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.11)),
        material=dark_bearing,
        name="bearing_pedestal",
    )
    support.visual(
        Cylinder(radius=0.09, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 2.19)),
        material=dark_bearing,
        name="bearing_cap",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 2.21)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
    )

    rotor = model.part("rotor_assembly")
    rotor.visual(
        Cylinder(radius=0.17, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=painted_steel,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.05, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_bearing,
        name="top_boss",
    )

    arm_reach = 0.63
    inner_frame_x = 0.15
    frame_top_z = 0.04
    frame_drop = 1.94
    frame_center_z = frame_top_z - (frame_drop * 0.5)
    vertical_radius = 0.012
    lower_bar_radius = 0.011
    top_arm_radius = 0.014
    lower_bar_length = arm_reach - inner_frame_x
    lower_bar_center_x = inner_frame_x + (lower_bar_length * 0.5)

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            Cylinder(radius=top_arm_radius, length=arm_reach),
            origin=Origin(
                xyz=(arm_reach * 0.5 * c, arm_reach * 0.5 * s, frame_top_z),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=galvanized_steel,
            name=f"arm_{index}_top",
        )
        rotor.visual(
            Cylinder(radius=vertical_radius, length=frame_drop),
            origin=Origin(
                xyz=(inner_frame_x * c, inner_frame_x * s, frame_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=galvanized_steel,
            name=f"arm_{index}_inner_post",
        )
        rotor.visual(
            Cylinder(radius=vertical_radius, length=frame_drop),
            origin=Origin(
                xyz=(arm_reach * c, arm_reach * s, frame_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=galvanized_steel,
            name=f"arm_{index}_outer_post",
        )
        for bar_level, z in enumerate((-0.44, -0.92, -1.40, -1.88)):
            rotor.visual(
                Cylinder(radius=lower_bar_radius, length=lower_bar_length),
                origin=Origin(
                    xyz=(lower_bar_center_x * c, lower_bar_center_x * s, z),
                    rpy=(0.0, math.pi / 2.0, angle),
                ),
                material=galvanized_steel,
                name=f"arm_{index}_bar_{bar_level}",
            )
    rotor.inertial = Inertial.from_geometry(
        Box((1.30, 1.30, 2.05)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, -0.82)),
    )

    model.articulation(
        "post_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    support = object_model.get_part("support_post")
    rotor = object_model.get_part("rotor_assembly")
    spin = object_model.get_articulation("post_to_rotor")
    bearing_cap = support.get_visual("bearing_cap")
    rotor_hub = rotor.get_visual("rotor_hub")

    def elem_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.check(
        "continuous top bearing uses vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={spin.motion_limits}"
        ),
    )
    ctx.expect_contact(
        rotor,
        support,
        elem_a=rotor_hub,
        elem_b=bearing_cap,
        contact_tol=0.0005,
        name="rotor hub bears on the post cap at the top bearing",
    )

    arm_centers = [elem_center(rotor, f"arm_{index}_top") for index in range(3)]
    if all(center is not None for center in arm_centers):
        angles = sorted((math.atan2(center[1], center[0]) + 2.0 * math.pi) % (2.0 * math.pi) for center in arm_centers)
        gaps = [
            (angles[(i + 1) % 3] - angles[i]) % (2.0 * math.pi)
            for i in range(3)
        ]
        equally_spaced = all(abs(gap - (2.0 * math.pi / 3.0)) < 0.12 for gap in gaps)
    else:
        equally_spaced = False
        gaps = arm_centers
    ctx.check(
        "three rotor arms are equally spaced around the post",
        equally_spaced,
        details=f"arm_centers={arm_centers}, angle_gaps={gaps}",
    )

    rest_center = elem_center(rotor, "arm_0_top")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_center = elem_center(rotor, "arm_0_top")
    ctx.check(
        "rotor assembly turns about the vertical post axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.25
        and abs(rest_center[1]) < 0.05
        and turned_center[1] > 0.25
        and abs(turned_center[0]) < 0.05,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    support_aabb = ctx.part_world_aabb(support)
    rotor_aabb = ctx.part_world_aabb(rotor)
    if support_aabb is not None and rotor_aabb is not None:
        min_z = min(support_aabb[0][2], rotor_aabb[0][2])
        max_z = max(support_aabb[1][2], rotor_aabb[1][2])
        rotor_bottom = rotor_aabb[0][2]
        full_height_ok = max_z > 2.25 and rotor_bottom < 0.40 and min_z < 0.01
        full_height_details = f"min_z={min_z:.3f}, max_z={max_z:.3f}, rotor_bottom={rotor_bottom:.3f}"
    else:
        full_height_ok = False
        full_height_details = f"support_aabb={support_aabb}, rotor_aabb={rotor_aabb}"
    ctx.check(
        "turnstile keeps a full-height barrier silhouette",
        full_height_ok,
        details=full_height_details,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
