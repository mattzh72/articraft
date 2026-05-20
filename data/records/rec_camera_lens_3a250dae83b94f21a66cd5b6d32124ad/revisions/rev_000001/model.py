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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_barrel_shell():
    outer_profile = [
        (0.036, 0.008),
        (0.039, 0.012),
        (0.041, 0.022),
        (0.0422, 0.030),
        (0.0422, 0.056),
        (0.0455, 0.068),
        (0.0490, 0.078),
        (0.0510, 0.082),
    ]
    inner_profile = [
        (0.0260, 0.008),
        (0.0285, 0.014),
        (0.0305, 0.028),
        (0.0315, 0.056),
        (0.0325, 0.070),
        (0.0340, 0.082),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_focus_ring_shell():
    outer_profile = [
        (0.0475, -0.011),
        (0.0508, -0.011),
        (0.0508, -0.0090),
        (0.0488, -0.0075),
        (0.0506, -0.0055),
        (0.0488, -0.0035),
        (0.0506, -0.0015),
        (0.0488, 0.0000),
        (0.0506, 0.0020),
        (0.0488, 0.0040),
        (0.0506, 0.0060),
        (0.0488, 0.0080),
        (0.0508, 0.0100),
        (0.0508, 0.0110),
        (0.0475, 0.0110),
    ]
    inner_profile = [
        (0.0422, -0.0110),
        (0.0422, 0.0110),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_front_element():
    return LatheGeometry(
        [
            (0.0, -0.006),
            (0.022, -0.006),
            (0.030, -0.003),
            (0.034, 0.000),
            (0.040, 0.008),
            (0.044, 0.018),
            (0.046, 0.027),
            (0.043, 0.031),
            (0.030, 0.032),
            (0.0, 0.028),
        ],
        segments=72,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fisheye_prime_lens")

    barrel_black = model.material("barrel_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.17, 0.22, 0.28, 0.55))
    filter_frame = model.material("filter_frame", rgba=(0.12, 0.12, 0.13, 1.0))
    gel_tint = model.material("gel_tint", rgba=(0.26, 0.34, 0.30, 0.50))

    barrel_body = model.part("barrel_body")
    barrel_body.visual(
        mesh_from_geometry(_build_barrel_shell(), "fisheye_barrel_shell"),
        material=barrel_black,
        name="barrel_shell",
    )
    barrel_body.inertial = Inertial.from_geometry(
        Box((0.102, 0.102, 0.084)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(_build_focus_ring_shell(), "fisheye_focus_ring"),
        material=rubber_black,
        name="focus_grip",
    )
    focus_ring.visual(
        Box((0.008, 0.0025, 0.005)),
        origin=Origin(xyz=(0.0, 0.0510, 0.0)),
        material=mount_metal,
        name="focus_marker",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.051, length=0.022),
        mass=0.08,
    )

    front_element = model.part("front_element")
    front_element.visual(
        mesh_from_geometry(_build_front_element(), "fisheye_front_element"),
        material=coated_glass,
        name="front_glass",
    )
    front_element.visual(
        Cylinder(radius=0.040, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=barrel_black,
        name="front_retainer_flange",
    )
    front_element.visual(
        Cylinder(radius=0.0338, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=barrel_black,
        name="front_cell_spigot",
    )
    front_element.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.038),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    rear_mount = model.part("rear_mount")
    rear_mount.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=mount_metal,
        name="mount_flange",
    )
    rear_mount.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=mount_metal,
        name="mount_throat",
    )
    for index, angle in enumerate((math.radians(60.0), math.pi, math.radians(300.0))):
        rear_mount.visual(
            Box((0.012, 0.005, 0.002)),
            origin=Origin(
                xyz=(0.0275 * math.cos(angle), 0.0275 * math.sin(angle), 0.0048),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_metal,
            name=f"bayonet_lug_{index}",
        )
    rear_mount.visual(
        Box((0.006, 0.028, 0.012)),
        origin=Origin(xyz=(0.017, 0.0, 0.008)),
        material=mount_metal,
        name="filter_guide_bridge",
    )
    rear_mount.visual(
        Box((0.024, 0.026, 0.0025)),
        origin=Origin(xyz=(0.032, 0.0, 0.0115)),
        material=mount_metal,
        name="filter_guide_upper",
    )
    rear_mount.visual(
        Box((0.024, 0.026, 0.0025)),
        origin=Origin(xyz=(0.032, 0.0, 0.0045)),
        material=mount_metal,
        name="filter_guide_lower",
    )
    rear_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.014),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        Box((0.032, 0.022, 0.003)),
        material=filter_frame,
        name="drawer_tray",
    )
    filter_drawer.visual(
        Box((0.018, 0.014, 0.001)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0011)),
        material=gel_tint,
        name="gel_insert",
    )
    filter_drawer.visual(
        Box((0.006, 0.016, 0.005)),
        origin=Origin(xyz=(0.017, 0.0, 0.001)),
        material=filter_frame,
        name="drawer_tab",
    )
    filter_drawer.inertial = Inertial.from_geometry(
        Box((0.038, 0.026, 0.010)),
        mass=0.02,
    )

    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=barrel_body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.95,
            upper=0.95,
        ),
    )

    model.articulation(
        "barrel_to_front_element",
        ArticulationType.FIXED,
        parent=barrel_body,
        child=front_element,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    model.articulation(
        "barrel_to_rear_mount",
        ArticulationType.FIXED,
        parent=barrel_body,
        child=rear_mount,
        origin=Origin(),
    )

    model.articulation(
        "rear_mount_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=rear_mount,
        child=filter_drawer,
        origin=Origin(xyz=(0.020, 0.0, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.05,
            lower=0.0,
            upper=0.016,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_body = object_model.get_part("barrel_body")
    focus_ring = object_model.get_part("focus_ring")
    front_element = object_model.get_part("front_element")
    rear_mount = object_model.get_part("rear_mount")
    filter_drawer = object_model.get_part("filter_drawer")

    focus_joint = object_model.get_articulation("barrel_to_focus_ring")
    drawer_joint = object_model.get_articulation("rear_mount_to_filter_drawer")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        barrel_body,
        focus_ring,
        elem_a="barrel_shell",
        elem_b="focus_grip",
        reason=(
            "The manual focus ring is intentionally represented as a close-fitting "
            "rotating annular sleeve around the barrel core; the mesh-derived "
            "collision proxy treats the supported sleeve as overlapping the barrel shell."
        ),
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "focus ring rotates around optical axis",
        tuple(round(value, 4) for value in focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "filter drawer slides laterally",
        tuple(round(value, 4) for value in drawer_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={drawer_joint.axis}",
    )

    ctx.expect_contact(
        barrel_body,
        rear_mount,
        elem_a="barrel_shell",
        elem_b="mount_throat",
        name="rear mount seats against the barrel shell",
    )
    ctx.expect_contact(
        front_element,
        barrel_body,
        elem_a="front_retainer_flange",
        elem_b="barrel_shell",
        name="front element seats into the front barrel lip",
    )
    ctx.expect_origin_distance(
        focus_ring,
        barrel_body,
        axes="xy",
        max_dist=0.001,
        name="focus ring stays concentric with the barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel_body,
        axes="xy",
        min_overlap=0.080,
        name="focus ring wraps the barrel mid-section",
    )
    ctx.expect_within(
        filter_drawer,
        rear_mount,
        axes="yz",
        margin=0.004,
        name="filter drawer stays aligned inside the rear guide envelope",
    )
    ctx.expect_overlap(
        filter_drawer,
        rear_mount,
        axes="x",
        min_overlap=0.020,
        name="filter drawer remains inserted in the rear guide at rest",
    )

    def _elem_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    marker_rest = _elem_center(focus_ring, "focus_marker")
    drawer_rest = ctx.part_world_position(filter_drawer)
    drawer_upper = drawer_joint.motion_limits.upper if drawer_joint.motion_limits is not None else None

    with ctx.pose({focus_joint: 0.75}):
        marker_turned = _elem_center(focus_ring, "focus_marker")
    ctx.check(
        "focus marker visibly rotates with the focus ring",
        marker_rest is not None
        and marker_turned is not None
        and marker_turned[0] < marker_rest[0] - 0.02
        and marker_turned[1] < marker_rest[1] - 0.008,
        details=f"rest={marker_rest}, turned={marker_turned}",
    )

    with ctx.pose({drawer_joint: drawer_upper if drawer_upper is not None else 0.016}):
        drawer_extended = ctx.part_world_position(filter_drawer)
        ctx.expect_within(
            filter_drawer,
            rear_mount,
            axes="yz",
            margin=0.004,
            name="extended filter drawer stays on the rear guide rails",
        )
        ctx.expect_overlap(
            filter_drawer,
            rear_mount,
            axes="x",
            min_overlap=0.012,
            name="extended filter drawer retains insertion in the guide",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no part overlaps with the drawer extended")
    ctx.check(
        "filter drawer extends outward from the rear mount",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.012,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
