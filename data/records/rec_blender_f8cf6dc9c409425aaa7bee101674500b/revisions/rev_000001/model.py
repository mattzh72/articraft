from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_handle_body_mesh() -> MeshGeometry:
    body = CapsuleGeometry(
        radius=0.026,
        length=0.176,
        radial_segments=30,
        height_segments=10,
    ).translate(0.0, 0.0, 0.126)
    lower_sleeve = CylinderGeometry(radius=0.0215, height=0.018, radial_segments=36).translate(
        0.0,
        0.0,
        0.023,
    )
    return _merge_geometries(body, lower_sleeve)


def _build_handle_socket_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0195, -0.014),
            (0.0230, -0.012),
            (0.0240, 0.000),
            (0.0225, 0.018),
        ],
        inner_profile=[
            (0.0185, -0.014),
            (0.0185, -0.010),
            (0.0185, 0.010),
            (0.0175, 0.016),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_trigger_pad_mesh() -> MeshGeometry:
    trigger_pad = CapsuleGeometry(
        radius=0.0045,
        length=0.064,
        radial_segments=24,
        height_segments=8,
    ).translate(0.0045, 0.0, -0.038)
    trigger_bridge = BoxGeometry((0.004, 0.010, 0.014)).translate(0.002, 0.0, -0.007)
    return _merge_geometries(trigger_pad, trigger_bridge)


def _build_shaft_core_mesh() -> MeshGeometry:
    coupling_spigot = CylinderGeometry(
        radius=0.0170,
        height=0.026,
        radial_segments=48,
    ).translate(0.0, 0.0, 0.001)
    coupling_collar = CylinderGeometry(
        radius=0.0205,
        height=0.010,
        radial_segments=48,
    ).translate(0.0, 0.0, -0.021)
    shaft_tube = CylinderGeometry(radius=0.0098, height=0.180, radial_segments=40).translate(
        0.0,
        0.0,
        -0.091,
    )
    drive_spindle = CylinderGeometry(
        radius=0.0035,
        height=0.030,
        radial_segments=28,
    ).translate(0.0, 0.0, -0.195)
    drive_washer = CylinderGeometry(
        radius=0.0085,
        height=0.002,
        radial_segments=28,
    ).translate(0.0, 0.0, -0.210)
    return _merge_geometries(coupling_spigot, coupling_collar, shaft_tube, drive_spindle, drive_washer)


def _build_bell_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0102, -0.176),
            (0.0125, -0.188),
            (0.0180, -0.201),
            (0.0260, -0.221),
            (0.0310, -0.236),
        ],
        inner_profile=[
            (0.0086, -0.176),
            (0.0105, -0.188),
            (0.0150, -0.201),
            (0.0215, -0.221),
        ],
        segments=60,
        start_cap="flat",
        end_cap="flat",
    )


def _build_blade_set_mesh() -> MeshGeometry:
    blade_set = MeshGeometry()
    for index in range(4):
        blade = BoxGeometry((0.015, 0.0048, 0.0014)).translate(
            0.006,
            0.0,
            0.001 if index % 2 == 0 else -0.001,
        )
        blade.rotate_y(0.22 if index % 2 == 0 else -0.22)
        blade.rotate_z(index * (math.pi / 2.0))
        blade_set.merge(blade)
    return blade_set


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="immersion_hand_blender")

    housing_black = model.material("housing_black", rgba=(0.12, 0.13, 0.14, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.44, 0.47, 1.0))

    motor_handle = model.part("motor_handle")
    motor_handle.visual(
        mesh_from_geometry(_build_handle_body_mesh(), "motor_handle_body"),
        material=housing_black,
        name="body_shell",
    )
    motor_handle.visual(
        mesh_from_geometry(_build_handle_socket_mesh(), "motor_handle_socket"),
        material=satin_steel,
        name="coupling_socket",
    )
    motor_handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.256),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )

    shaft_assembly = model.part("shaft_assembly")
    shaft_assembly.visual(
        mesh_from_geometry(_build_shaft_core_mesh(), "blender_shaft_core"),
        material=satin_steel,
        name="shaft_core",
    )
    shaft_assembly.visual(
        mesh_from_geometry(_build_bell_shell_mesh(), "blender_bell_shell"),
        material=satin_steel,
        name="bell_shell",
    )
    shaft_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.252),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
    )

    blade_rotor = model.part("blade_rotor")
    blade_rotor.visual(
        Cylinder(radius=0.0075, length=0.006),
        material=dark_steel,
        name="hub",
    )
    blade_rotor.visual(
        mesh_from_geometry(_build_blade_set_mesh(), "blender_blade_set"),
        material=dark_steel,
        name="blade_set",
    )
    blade_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.010),
        mass=0.04,
    )

    power_trigger = model.part("power_trigger")
    power_trigger.visual(
        Cylinder(radius=0.0025, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="pivot_barrel",
    )
    power_trigger.visual(
        mesh_from_geometry(_build_trigger_pad_mesh(), "blender_trigger_pad"),
        material=soft_black,
        name="trigger_pad",
    )
    power_trigger.inertial = Inertial.from_geometry(
        Box((0.012, 0.026, 0.078)),
        mass=0.03,
        origin=Origin(xyz=(0.0045, 0.0, -0.038)),
    )

    model.articulation(
        "handle_to_shaft_lock",
        ArticulationType.REVOLUTE,
        parent=motor_handle,
        child=shaft_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(24.0),
        ),
    )

    model.articulation(
        "shaft_to_blade_rotor",
        ArticulationType.CONTINUOUS,
        parent=shaft_assembly,
        child=blade_rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.214)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=90.0),
    )

    model.articulation(
        "handle_to_power_trigger",
        ArticulationType.REVOLUTE,
        parent=motor_handle,
        child=power_trigger,
        origin=Origin(xyz=(0.0285, 0.0, 0.184)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_handle = object_model.get_part("motor_handle")
    shaft_assembly = object_model.get_part("shaft_assembly")
    blade_rotor = object_model.get_part("blade_rotor")
    power_trigger = object_model.get_part("power_trigger")

    shaft_lock = object_model.get_articulation("handle_to_shaft_lock")
    blade_joint = object_model.get_articulation("shaft_to_blade_rotor")
    trigger_joint = object_model.get_articulation("handle_to_power_trigger")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
        "twist lock rotates about the blender axis",
        shaft_lock.axis == (0.0, 0.0, 1.0),
        details=f"axis={shaft_lock.axis}",
    )
    ctx.check(
        "blade rotor spins on the shaft axis",
        blade_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={blade_joint.axis}",
    )
    ctx.check(
        "power trigger pivots on a lateral pin",
        trigger_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={trigger_joint.axis}",
    )

    ctx.expect_within(
        shaft_assembly,
        motor_handle,
        axes="xy",
        inner_elem="shaft_core",
        outer_elem="coupling_socket",
        margin=0.002,
        name="shaft coupling stays centered in the handle socket",
    )
    ctx.expect_overlap(
        shaft_assembly,
        motor_handle,
        axes="z",
        elem_a="shaft_core",
        elem_b="coupling_socket",
        min_overlap=0.020,
        name="shaft coupling remains inserted into the handle socket",
    )
    ctx.expect_contact(
        power_trigger,
        motor_handle,
        elem_a="pivot_barrel",
        elem_b="body_shell",
        contact_tol=2e-4,
        name="trigger barrel is carried by the handle shell",
    )
    ctx.expect_contact(
        blade_rotor,
        shaft_assembly,
        elem_a="hub",
        elem_b="shaft_core",
        contact_tol=2e-4,
        name="blade hub rides on the shaft tip axle support",
    )
    ctx.expect_within(
        blade_rotor,
        shaft_assembly,
        axes="xy",
        inner_elem="blade_set",
        outer_elem="bell_shell",
        margin=0.0,
        name="blade sweep stays inside the guard diameter",
    )

    shaft_lock_upper = shaft_lock.motion_limits.upper if shaft_lock.motion_limits is not None else None
    trigger_upper = trigger_joint.motion_limits.upper if trigger_joint.motion_limits is not None else None

    if shaft_lock_upper is not None:
        with ctx.pose({shaft_lock: shaft_lock_upper}):
            ctx.expect_overlap(
                shaft_assembly,
                motor_handle,
                axes="z",
                elem_a="shaft_core",
                elem_b="coupling_socket",
                min_overlap=0.020,
                name="shaft remains inserted through twist-lock rotation",
            )

    trigger_rest_aabb = ctx.part_element_world_aabb(power_trigger, elem="trigger_pad")
    if trigger_upper is not None:
        with ctx.pose({trigger_joint: trigger_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pressed trigger pose clears the housing")
            ctx.expect_contact(
                power_trigger,
                motor_handle,
                elem_a="pivot_barrel",
                elem_b="body_shell",
                contact_tol=2e-4,
                name="trigger pivot stays seated when pressed",
            )
            trigger_pressed_aabb = ctx.part_element_world_aabb(power_trigger, elem="trigger_pad")
        ctx.check(
            "trigger travel moves inward toward the handle",
            trigger_rest_aabb is not None
            and trigger_pressed_aabb is not None
            and trigger_pressed_aabb[0][0] < trigger_rest_aabb[0][0] - 0.0015,
            details=f"rest={trigger_rest_aabb}, pressed={trigger_pressed_aabb}",
        )

    with ctx.pose({blade_joint: math.pi / 4.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated blade clears the bell guard")
        ctx.expect_contact(
            blade_rotor,
            shaft_assembly,
            elem_a="hub",
            elem_b="shaft_core",
            contact_tol=2e-4,
            name="blade hub stays seated during rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
