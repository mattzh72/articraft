from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_box(part, size, xyz, material, *, rpy=(0.0, 0.0, 0.0), name=None):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, material, *, axis="z", name=None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _build_drawer(
    model,
    *,
    name,
    z_base,
    face_height,
    shell_height,
    shell_material,
    face_material,
    handle_material,
    liner_material,
):
    drawer = model.part(name)

    floor_depth = 0.464
    total_depth = 0.488
    floor_width = 0.332
    body_width = 0.350
    face_width = 0.372

    _add_box(
        drawer,
        (floor_depth, floor_width, 0.014),
        (floor_depth / 2.0, 0.0, 0.007),
        shell_material,
        name=f"{name}_floor",
    )
    _add_box(
        drawer,
        (floor_depth, 0.012, shell_height),
        (floor_depth / 2.0, 0.169, shell_height / 2.0),
        shell_material,
        name=f"{name}_left_wall",
    )
    _add_box(
        drawer,
        (floor_depth, 0.012, shell_height),
        (floor_depth / 2.0, -0.169, shell_height / 2.0),
        shell_material,
        name=f"{name}_right_wall",
    )
    _add_box(
        drawer,
        (0.014, body_width, shell_height),
        (0.007, 0.0, shell_height / 2.0),
        shell_material,
        name=f"{name}_back_wall",
    )
    _add_box(
        drawer,
        (0.024, face_width, face_height),
        (total_depth - 0.012, 0.0, face_height / 2.0),
        face_material,
        name=f"{name}_face",
    )
    _add_box(
        drawer,
        (floor_depth - 0.030, floor_width - 0.018, 0.003),
        (floor_depth / 2.0 + 0.010, 0.0, 0.0155),
        liner_material,
        name=f"{name}_liner",
    )
    _add_box(
        drawer,
        (0.016, 0.014, 0.024),
        (total_depth + 0.001, 0.090, face_height * 0.50),
        handle_material,
        name=f"{name}_handle_left_post",
    )
    _add_box(
        drawer,
        (0.016, 0.014, 0.024),
        (total_depth + 0.001, -0.090, face_height * 0.50),
        handle_material,
        name=f"{name}_handle_right_post",
    )
    _add_cylinder(
        drawer,
        0.008,
        0.190,
        (total_depth + 0.016, 0.0, face_height * 0.50),
        handle_material,
        axis="y",
        name=f"{name}_handle_grip",
    )
    _add_box(
        drawer,
        (0.004, 0.110, 0.030),
        (total_depth - 0.001, 0.0, face_height * 0.76),
        handle_material,
        name=f"{name}_label_frame",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((total_depth, face_width, face_height)),
        mass=4.0 if face_height < 0.18 else 5.5,
        origin=Origin(xyz=(total_depth / 2.0, 0.0, face_height / 2.0)),
    )

    model.articulation(
        f"{name}_slide",
        ArticulationType.PRISMATIC,
        parent="body",
        child=name,
        origin=Origin(xyz=(-0.216, 0.0, z_base)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.40, lower=0.0, upper=0.230),
    )

    return drawer


def _build_caster_fork(model, *, name, locking, material, lever_material):
    fork = model.part(name)
    _add_cylinder(fork, 0.007, 0.050, (0.0, 0.0, -0.025), material, name=f"{name}_stem")
    _add_box(fork, (0.050, 0.060, 0.008), (0.0, 0.0, -0.054), material, name=f"{name}_top_plate")
    _add_box(fork, (0.045, 0.006, 0.052), (0.0, 0.019, -0.084), material, name=f"{name}_left_fork")
    _add_box(
        fork, (0.045, 0.006, 0.052), (0.0, -0.019, -0.084), material, name=f"{name}_right_fork"
    )
    _add_box(
        fork, (0.014, 0.007, 0.014), (0.0, 0.0125, -0.110), material, name=f"{name}_left_axle_boss"
    )
    _add_box(
        fork,
        (0.014, 0.007, 0.014),
        (0.0, -0.0125, -0.110),
        material,
        name=f"{name}_right_axle_boss",
    )
    _add_box(
        fork, (0.022, 0.032, 0.008), (0.026, 0.0, -0.050), material, name=f"{name}_front_bridge"
    )
    if locking:
        _add_box(
            fork,
            (0.020, 0.024, 0.006),
            (0.029, 0.0, -0.043),
            lever_material,
            name=f"{name}_lock_pedal",
        )

    fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.060, 0.110)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )
    return fork


def _build_caster_wheel(model, *, name, tire_material, hub_material):
    wheel = model.part(name)
    _add_cylinder(
        wheel, 0.050, 0.018, (0.0, 0.0, 0.0), tire_material, axis="y", name=f"{name}_tire"
    )
    _add_cylinder(wheel, 0.032, 0.014, (0.0, 0.0, 0.0), hub_material, axis="y", name=f"{name}_hub")
    _add_cylinder(
        wheel, 0.014, 0.003, (0.0, 0.0075, 0.0), hub_material, axis="y", name=f"{name}_outer_cap"
    )
    _add_cylinder(
        wheel, 0.014, 0.003, (0.0, -0.0075, 0.0), hub_material, axis="y", name=f"{name}_inner_cap"
    )
    wheel.inertial = Inertial.from_geometry(
        Box((0.100, 0.018, 0.100)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_cart", assets=ASSETS)

    body_red = Material("powder_coat_red", (0.72, 0.10, 0.08, 1.0))
    charcoal = Material("charcoal_steel", (0.18, 0.19, 0.22, 1.0))
    steel = Material("brushed_steel", (0.72, 0.74, 0.76, 1.0))
    rubber = Material("rubber_black", (0.10, 0.10, 0.11, 1.0))
    black = Material("matte_black", (0.16, 0.16, 0.17, 1.0))
    model.materials.extend([body_red, charcoal, steel, rubber, black])

    body = model.part("body")
    _add_box(body, (0.060, 0.060, 0.020), (0.170, 0.160, 0.170), charcoal, name="front_left_mount")
    _add_box(
        body, (0.060, 0.060, 0.020), (0.170, -0.160, 0.170), charcoal, name="front_right_mount"
    )
    _add_box(body, (0.060, 0.060, 0.020), (-0.210, 0.160, 0.170), charcoal, name="rear_left_mount")
    _add_box(
        body, (0.060, 0.060, 0.020), (-0.210, -0.160, 0.170), charcoal, name="rear_right_mount"
    )
    _add_box(
        body, (0.460, 0.340, 0.020), (-0.020, 0.0, 0.190), charcoal, name="undercarriage_plate"
    )
    _add_box(body, (0.500, 0.384, 0.018), (-0.020, 0.0, 0.209), charcoal, name="bottom_shelf")
    _add_box(body, (0.500, 0.384, 0.016), (-0.020, 0.0, 0.498), charcoal, name="middle_shelf")
    _add_box(body, (0.520, 0.018, 0.620), (-0.020, 0.201, 0.520), body_red, name="left_side")
    _add_box(body, (0.520, 0.018, 0.620), (-0.020, -0.201, 0.520), body_red, name="right_side")
    _add_box(body, (0.018, 0.384, 0.620), (-0.271, 0.0, 0.520), body_red, name="rear_panel")
    _add_box(body, (0.520, 0.420, 0.020), (-0.020, 0.0, 0.840), body_red, name="top_tray_floor")
    _add_box(body, (0.520, 0.018, 0.055), (-0.020, 0.201, 0.8575), body_red, name="top_left_rail")
    _add_box(body, (0.520, 0.018, 0.055), (-0.020, -0.201, 0.8575), body_red, name="top_right_rail")
    _add_box(body, (0.018, 0.384, 0.055), (-0.271, 0.0, 0.8575), body_red, name="top_rear_rail")
    _add_box(body, (0.018, 0.384, 0.055), (0.231, 0.0, 0.8575), body_red, name="top_front_rail")
    _add_box(body, (0.010, 0.018, 0.620), (0.229, 0.183, 0.520), body_red, name="front_left_stile")
    _add_box(
        body, (0.010, 0.018, 0.620), (0.229, -0.183, 0.520), body_red, name="front_right_stile"
    )
    _add_box(body, (0.010, 0.348, 0.040), (0.229, 0.0, 0.190), body_red, name="front_lower_rail")
    _add_box(body, (0.010, 0.348, 0.020), (0.229, 0.0, 0.458), body_red, name="front_center_rail")
    _add_box(body, (0.010, 0.348, 0.052), (0.229, 0.0, 0.700), body_red, name="front_header")
    _add_box(body, (0.300, 0.022, 0.012), (-0.035, 0.189, 0.228), steel, name="lower_left_runner")
    _add_box(body, (0.300, 0.022, 0.012), (-0.035, -0.189, 0.228), steel, name="lower_right_runner")
    _add_box(body, (0.300, 0.022, 0.012), (-0.035, 0.189, 0.516), steel, name="upper_left_runner")
    _add_box(body, (0.300, 0.022, 0.012), (-0.035, -0.189, 0.516), steel, name="upper_right_runner")
    _add_box(
        body, (0.018, 0.038, 0.022), (0.105, 0.219, 0.600), steel, name="handle_front_standoff"
    )
    _add_box(
        body, (0.018, 0.038, 0.022), (-0.105, 0.219, 0.600), steel, name="handle_rear_standoff"
    )
    _add_cylinder(body, 0.009, 0.210, (0.0, 0.247, 0.600), steel, axis="x", name="side_push_handle")
    _add_box(body, (0.470, 0.340, 0.003), (-0.010, 0.0, 0.8515), black, name="top_tray_liner")
    _add_box(body, (0.340, 0.004, 0.030), (-0.030, 0.207, 0.620), charcoal, name="left_stamp_rib")
    _add_box(body, (0.340, 0.004, 0.030), (-0.030, -0.207, 0.620), charcoal, name="right_stamp_rib")

    body.inertial = Inertial.from_geometry(
        Box((0.520, 0.420, 0.720)),
        mass=38.0,
        origin=Origin(xyz=(-0.020, 0.0, 0.520)),
    )

    _build_drawer(
        model,
        name="upper_drawer",
        z_base=0.510,
        face_height=0.160,
        shell_height=0.100,
        shell_material=charcoal,
        face_material=body_red,
        handle_material=steel,
        liner_material=black,
    )
    _build_drawer(
        model,
        name="lower_drawer",
        z_base=0.222,
        face_height=0.210,
        shell_height=0.145,
        shell_material=charcoal,
        face_material=body_red,
        handle_material=steel,
        liner_material=black,
    )

    caster_mounts = {
        "front_left": (0.170, 0.160, True),
        "front_right": (0.170, -0.160, True),
        "rear_left": (-0.210, 0.160, False),
        "rear_right": (-0.210, -0.160, False),
    }
    for prefix, (x_pos, y_pos, locking) in caster_mounts.items():
        fork_name = f"{prefix}_caster_fork"
        wheel_name = f"{prefix}_caster_wheel"
        _build_caster_fork(
            model, name=fork_name, locking=locking, material=steel, lever_material=black
        )
        _build_caster_wheel(model, name=wheel_name, tire_material=rubber, hub_material=charcoal)
        model.articulation(
            f"{prefix}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent="body",
            child=fork_name,
            origin=Origin(xyz=(x_pos, y_pos, 0.160)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=45.0, velocity=8.0),
        )
        model.articulation(
            f"{prefix}_wheel_roll",
            ArticulationType.CONTINUOUS,
            parent=fork_name,
            child=wheel_name,
            origin=Origin(xyz=(0.0, 0.0, -0.110)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "front_left_caster_fork",
        "front_left_caster_wheel",
        reason="caster wheel runs between fork arms and around the axle bosses",
    )
    ctx.allow_overlap(
        "front_right_caster_fork",
        "front_right_caster_wheel",
        reason="caster wheel runs between fork arms and around the axle bosses",
    )
    ctx.allow_overlap(
        "rear_left_caster_fork",
        "rear_left_caster_wheel",
        reason="caster wheel runs between fork arms and around the axle bosses",
    )
    ctx.allow_overlap(
        "rear_right_caster_fork",
        "rear_right_caster_wheel",
        reason="caster wheel runs between fork arms and around the axle bosses",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("upper_drawer", "body", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_overlap("lower_drawer", "body", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("upper_drawer", "lower_drawer", axis="z", max_gap=0.10, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "upper_drawer_slide", "upper_drawer", world_axis="x", direction="positive", min_delta=0.08
    )
    ctx.expect_joint_motion_axis(
        "lower_drawer_slide", "lower_drawer", world_axis="x", direction="positive", min_delta=0.08
    )

    for fork_name in (
        "front_left_caster_fork",
        "front_right_caster_fork",
        "rear_left_caster_fork",
        "rear_right_caster_fork",
    ):
        ctx.expect_aabb_gap("body", fork_name, axis="z", max_gap=0.006, max_penetration=0.0)

    for wheel_name in (
        "front_left_caster_wheel",
        "front_right_caster_wheel",
        "rear_left_caster_wheel",
        "rear_right_caster_wheel",
    ):
        ctx.expect_aabb_overlap(wheel_name, "body", axes="xy", min_overlap=0.010)

    with ctx.pose(upper_drawer_slide=0.230):
        ctx.expect_aabb_overlap("upper_drawer", "body", axes="xy", min_overlap=0.10)

    with ctx.pose(lower_drawer_slide=0.230):
        ctx.expect_aabb_overlap("lower_drawer", "body", axes="xy", min_overlap=0.10)

    with ctx.pose(upper_drawer_slide=0.230, lower_drawer_slide=0.230):
        ctx.expect_aabb_overlap("upper_drawer", "body", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_overlap("lower_drawer", "body", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_gap("upper_drawer", "lower_drawer", axis="z", max_gap=0.10, max_penetration=0.0)

    with ctx.pose(
        front_left_caster_swivel=math.pi / 4.0,
        front_right_caster_swivel=-math.pi / 4.0,
        rear_left_caster_swivel=math.pi / 6.0,
        rear_right_caster_swivel=-math.pi / 6.0,
        front_left_wheel_roll=1.2,
        front_right_wheel_roll=-0.8,
    ):
        ctx.expect_aabb_gap("body", "front_left_caster_fork", axis="z", max_gap=0.006, max_penetration=0.0)
        ctx.expect_aabb_gap("body", "front_right_caster_fork", axis="z", max_gap=0.006, max_penetration=0.0)
        ctx.expect_aabb_overlap("front_left_caster_wheel", "body", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_overlap("front_right_caster_wheel", "body", axes="xy", min_overlap=0.010)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
