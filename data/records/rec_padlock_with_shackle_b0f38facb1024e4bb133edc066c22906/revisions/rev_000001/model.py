from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luggage_padlock")

    body_w = 0.034
    body_d = 0.010
    wheel_radius = 0.0039
    wheel_length = 0.0062
    wheel_centers_x = (-0.0085, 0.0, 0.0085)
    wheel_center_y = 0.0058
    wheel_center_z = 0.0049
    rod_radius = 0.0013
    shackle_leg_spacing = 0.021

    body_metal = model.material("body_metal", rgba=(0.76, 0.77, 0.74, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.88, 0.89, 0.90, 1.0))
    dial_black = model.material("dial_black", rgba=(0.11, 0.12, 0.13, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.91, 0.92, 0.90, 1.0))

    body = model.part("body")
    upper_shell_geom = ExtrudeGeometry(
        rounded_rect_profile(body_w, body_d, 0.0032),
        0.020,
        center=True,
    )
    body.visual(
        mesh_from_geometry(upper_shell_geom, "padlock_upper_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=body_metal,
        name="upper_shell",
    )
    top_cap_geom = ExtrudeGeometry(
        rounded_rect_profile(0.024, body_d, 0.0026),
        0.007,
        center=True,
    )
    body.visual(
        mesh_from_geometry(top_cap_geom, "padlock_top_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0340)),
        material=body_metal,
        name="top_cap",
    )
    body.visual(
        Box((body_w, 0.004, 0.0105)),
        origin=Origin(xyz=(0.0, -0.003, 0.00525)),
        material=body_metal,
        name="lower_back",
    )
    body.visual(
        Box((0.029, 0.001, 0.0090)),
        origin=Origin(xyz=(0.0, -0.0005, 0.0060)),
        material=body_metal,
        name="dial_backplate",
    )
    body.visual(
        Box((body_w, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.001, 0.001)),
        material=body_metal,
        name="lower_floor",
    )
    body.visual(
        Box((body_w, 0.003, 0.0045)),
        origin=Origin(xyz=(0.0, 0.0045, 0.01025)),
        material=body_metal,
        name="dial_visor",
    )
    body.visual(
        Box((0.004, 0.006, 0.0105)),
        origin=Origin(xyz=(-0.015, 0.003, 0.00525)),
        material=body_metal,
        name="left_cheek",
    )
    body.visual(
        Box((0.004, 0.006, 0.0105)),
        origin=Origin(xyz=(0.015, 0.003, 0.00525)),
        material=body_metal,
        name="right_cheek",
    )
    body.visual(
        Box((0.002, 0.006, 0.0105)),
        origin=Origin(xyz=(-0.00425, 0.003, 0.00525)),
        material=body_metal,
        name="left_separator",
    )
    body.visual(
        Box((0.002, 0.006, 0.0105)),
        origin=Origin(xyz=(0.00425, 0.003, 0.00525)),
        material=body_metal,
        name="right_separator",
    )
    body.visual(
        Cylinder(radius=0.0024, length=0.003),
        origin=Origin(xyz=(-0.0105, 0.0, 0.0390)),
        material=body_metal,
        name="left_socket",
    )
    body.visual(
        Cylinder(radius=0.0024, length=0.003),
        origin=Origin(xyz=(0.0105, 0.0, 0.0390)),
        material=body_metal,
        name="right_socket",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, 0.041)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
    )

    shackle = model.part("shackle")
    shackle_path = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.007),
            (0.0015, 0.0, 0.015),
            (0.0050, 0.0, 0.022),
            (0.0105, 0.0, 0.0245),
            (0.0160, 0.0, 0.0235),
            (0.0195, 0.0, 0.0180),
            (shackle_leg_spacing, 0.0, 0.0100),
            (shackle_leg_spacing, 0.0, 0.0),
        ],
        radius=rod_radius,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    shackle.visual(
        mesh_from_geometry(shackle_path, "padlock_shackle"),
        material=shackle_steel,
        name="shackle_bar",
    )
    shackle.visual(
        Sphere(radius=rod_radius * 1.04),
        origin=Origin(xyz=(shackle_leg_spacing, 0.0, 0.0)),
        material=shackle_steel,
        name="free_tip",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.024, 0.003, 0.026)),
        mass=0.014,
        origin=Origin(xyz=(0.0105, 0.0, 0.0125)),
    )

    for dial_index, x_center in enumerate(wheel_centers_x, start=1):
        dial = model.part(f"dial_{dial_index}")
        dial.visual(
            Cylinder(radius=wheel_radius, length=wheel_length),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dial_black,
            name="wheel",
        )
        dial.visual(
            Cylinder(radius=wheel_radius * 1.08, length=0.0012),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dial_black,
            name="wheel_rim",
        )
        dial.visual(
            Box((0.0014, 0.0011, 0.0017)),
            origin=Origin(xyz=(0.0, 0.0025, 0.0023)),
            material=dial_mark,
            name="indicator",
        )
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius * 1.08, length=wheel_length),
            mass=0.006,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )

        model.articulation(
            f"body_to_dial_{dial_index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x_center, wheel_center_y, wheel_center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.15, velocity=16.0),
        )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-0.0105, 0.0, 0.0418)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=0.0, upper=1.15),
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

    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    shackle_hinge = object_model.get_articulation("body_to_shackle")
    dial_parts = [object_model.get_part(f"dial_{index}") for index in (1, 2, 3)]
    dial_joints = [object_model.get_articulation(f"body_to_dial_{index}") for index in (1, 2, 3)]

    ctx.check(
        "shackle hinge opens upward around the retained leg",
        shackle_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={shackle_hinge.axis}",
    )
    for index, joint in enumerate(dial_joints, start=1):
        ctx.check(
            f"dial {index} spins on its local x axis",
            joint.axis == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    with ctx.pose({shackle_hinge: 0.0}):
        ctx.expect_contact(
            shackle,
            body,
            elem_a="free_tip",
            elem_b="right_socket",
            contact_tol=0.0002,
            name="closed shackle seats on the free socket",
        )

    with ctx.pose({shackle_hinge: 1.0}):
        ctx.expect_gap(
            shackle,
            body,
            axis="z",
            positive_elem="free_tip",
            negative_elem="right_socket",
            min_gap=0.006,
            name="opened shackle lifts the free leg clear of the body",
        )

    for index, dial in enumerate(dial_parts, start=1):
        ctx.expect_overlap(
            dial,
            body,
            axes="xz",
            elem_a="wheel",
            min_overlap=0.003,
            name=f"dial {index} stays nested in the lower body face",
        )

    rest_indicator_centers = {
        dial.name: _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
        for dial in dial_parts
    }
    for joint, dial in zip(dial_joints, dial_parts):
        with ctx.pose({joint: pi / 2.0}):
            moved_center = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
            other_centers = {
                other.name: _aabb_center(ctx.part_element_world_aabb(other, elem="indicator"))
                for other in dial_parts
                if other.name != dial.name
            }
        rest_center = rest_indicator_centers[dial.name]
        moved_ok = (
            rest_center is not None
            and moved_center is not None
            and (
                ((moved_center[1] - rest_center[1]) ** 2 + (moved_center[2] - rest_center[2]) ** 2) ** 0.5
                > 0.003
            )
        )
        others_steady = all(
            rest_indicator_centers[name] is not None
            and center is not None
            and abs(center[1] - rest_indicator_centers[name][1]) < 1e-6
            and abs(center[2] - rest_indicator_centers[name][2]) < 1e-6
            for name, center in other_centers.items()
        )
        ctx.check(
            f"{dial.name} rotates independently",
            moved_ok and others_steady,
            details=(
                f"rest={rest_center}, moved={moved_center}, "
                f"others={other_centers}, rest_others={rest_indicator_centers}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
