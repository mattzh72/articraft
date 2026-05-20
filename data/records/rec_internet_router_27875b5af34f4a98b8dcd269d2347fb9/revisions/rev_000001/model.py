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


def _add_router_antenna(
    model: ArticulatedObject,
    *,
    name: str,
    parent,
    mount_x: float,
    mount_y: float,
    mount_z: float,
    material,
) -> None:
    antenna = model.part(name)
    antenna.visual(
        Cylinder(radius=0.0055, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="hinge_barrel",
    )
    antenna.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=material,
        name="neck",
    )
    antenna.visual(
        Box((0.020, 0.008, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.0655)),
        material=material,
        name="blade",
    )
    antenna.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.113), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="tip_cap",
    )
    antenna.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.125)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
    )

    model.articulation(
        f"housing_to_{name}",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=antenna,
        origin=Origin(xyz=(mount_x, mount_y, mount_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-1.20,
            upper=1.10,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_router")

    housing_color = model.material("housing_color", rgba=(0.15, 0.16, 0.17, 1.0))
    trim_color = model.material("trim_color", rgba=(0.11, 0.12, 0.13, 1.0))
    antenna_color = model.material("antenna_color", rgba=(0.08, 0.08, 0.09, 1.0))
    port_color = model.material("port_color", rgba=(0.03, 0.03, 0.04, 1.0))

    body_w = 0.260
    body_d = 0.180
    body_h = 0.036
    wall_t = 0.0025
    top_t = 0.0030

    door_w = 0.026
    door_h = 0.015
    door_t = 0.003
    door_center_x = 0.080
    door_hinge_x = door_center_x - (door_w / 2.0)
    door_z = 0.018

    housing = model.part("housing")
    housing.visual(
        Box((body_w, body_d, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - (top_t / 2.0))),
        material=housing_color,
        name="top_shell",
    )
    housing.visual(
        Box((wall_t, body_d, body_h - top_t)),
        origin=Origin(
            xyz=(-(body_w / 2.0) + (wall_t / 2.0), 0.0, (body_h - top_t) / 2.0)
        ),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, body_d, body_h - top_t)),
        origin=Origin(
            xyz=((body_w / 2.0) - (wall_t / 2.0), 0.0, (body_h - top_t) / 2.0)
        ),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((body_w - (2.0 * wall_t), wall_t, body_h - top_t)),
        origin=Origin(
            xyz=(0.0, -(body_d / 2.0) + (wall_t / 2.0), (body_h - top_t) / 2.0)
        ),
        material=housing_color,
        name="front_wall",
    )
    housing.visual(
        Box((body_w - (2.0 * wall_t), wall_t, body_h - top_t)),
        origin=Origin(
            xyz=(0.0, (body_d / 2.0) - (wall_t / 2.0), (body_h - top_t) / 2.0)
        ),
        material=housing_color,
        name="rear_wall",
    )
    housing.visual(
        Box((body_w - (2.0 * wall_t), body_d - (2.0 * wall_t), wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t / 2.0)),
        material=housing_color,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.220, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.078, 0.032)),
        material=trim_color,
        name="rear_spine",
    )
    for mount_name, mount_x in (
        ("left_mount", -0.085),
        ("center_mount", 0.0),
        ("right_mount", 0.085),
    ):
        housing.visual(
            Box((0.030, 0.014, 0.006)),
            origin=Origin(xyz=(mount_x, 0.083, 0.033)),
            material=trim_color,
            name=mount_name,
        )
    housing.visual(
        Box((0.024, 0.005, 0.011)),
        origin=Origin(xyz=(door_center_x, -0.0875, door_z)),
        material=port_color,
        name="usb_port_body",
    )
    housing.visual(
        Box((door_w + 0.010, 0.0012, door_h + 0.008)),
        origin=Origin(xyz=(door_center_x, -(body_d / 2.0) - 0.0006, door_z)),
        material=trim_color,
        name="usb_bezel",
    )
    for foot_name, foot_x, foot_y in (
        ("foot_front_left", -0.090, -0.060),
        ("foot_front_right", 0.090, -0.060),
        ("foot_rear_left", -0.090, 0.060),
        ("foot_rear_right", 0.090, 0.060),
    ):
        housing.visual(
            Box((0.024, 0.014, 0.003)),
            origin=Origin(xyz=(foot_x, foot_y, -0.0015)),
            material=trim_color,
            name=foot_name,
        )
    housing.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=1.10,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    usb_cover = model.part("usb_cover")
    usb_cover.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, -(door_t / 2.0), 0.0)),
        material=trim_color,
        name="cover_panel",
    )
    usb_cover.visual(
        Cylinder(radius=0.0015, length=door_h),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material=trim_color,
        name="hinge_knuckle",
    )
    usb_cover.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=0.02,
        origin=Origin(xyz=(door_w / 2.0, -(door_t / 2.0), 0.0)),
    )

    model.articulation(
        "housing_to_usb_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=usb_cover,
        origin=Origin(xyz=(door_hinge_x, -(body_d / 2.0), door_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    _add_router_antenna(
        model,
        name="antenna_left",
        parent=housing,
        mount_x=-0.085,
        mount_y=0.083,
        mount_z=0.0415,
        material=antenna_color,
    )
    _add_router_antenna(
        model,
        name="antenna_center",
        parent=housing,
        mount_x=0.0,
        mount_y=0.083,
        mount_z=0.0415,
        material=antenna_color,
    )
    _add_router_antenna(
        model,
        name="antenna_right",
        parent=housing,
        mount_x=0.085,
        mount_y=0.083,
        mount_z=0.0415,
        material=antenna_color,
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
    housing = object_model.get_part("housing")
    usb_cover = object_model.get_part("usb_cover")
    antenna_left = object_model.get_part("antenna_left")
    antenna_center = object_model.get_part("antenna_center")
    antenna_right = object_model.get_part("antenna_right")

    usb_hinge = object_model.get_articulation("housing_to_usb_cover")
    left_hinge = object_model.get_articulation("housing_to_antenna_left")
    center_hinge = object_model.get_articulation("housing_to_antenna_center")
    right_hinge = object_model.get_articulation("housing_to_antenna_right")

    def aabb_center(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        return tuple((bounds[0][index] + bounds[1][index]) / 2.0 for index in range(3))

    ctx.check(
        "router assembly parts exist",
        all(
            part is not None
            for part in (
                housing,
                usb_cover,
                antenna_left,
                antenna_center,
                antenna_right,
            )
        ),
        details="Expected housing, usb cover, and three antenna parts.",
    )

    with ctx.pose({usb_hinge: 0.0}):
        ctx.expect_contact(
            usb_cover,
            housing,
            contact_tol=0.0005,
            name="usb cover seats against the front panel",
        )
        ctx.expect_overlap(
            usb_cover,
            housing,
            axes="xz",
            min_overlap=0.010,
            name="usb cover aligns with the front opening footprint",
        )

    closed_cover_pos = aabb_center(usb_cover, "cover_panel")
    with ctx.pose({usb_hinge: 1.20}):
        opened_cover_pos = aabb_center(usb_cover, "cover_panel")
    ctx.check(
        "usb cover opens outward from the front face",
        closed_cover_pos is not None
        and opened_cover_pos is not None
        and opened_cover_pos[1] < closed_cover_pos[1] - 0.010,
        details=f"closed={closed_cover_pos}, open={opened_cover_pos}",
    )

    for antenna, hinge, label in (
        (antenna_left, left_hinge, "left"),
        (antenna_center, center_hinge, "center"),
        (antenna_right, right_hinge, "right"),
    ):
        with ctx.pose({hinge: 0.0}):
            ctx.expect_contact(
                antenna,
                housing,
                contact_tol=0.0006,
                name=f"{label} antenna sits on the rear mount",
            )

        rest_pos = aabb_center(antenna, "blade")
        with ctx.pose({hinge: 0.85}):
            raised_pos = aabb_center(antenna, "blade")
        ctx.check(
            f"{label} antenna rotates rearward on its hinge",
            rest_pos is not None
            and raised_pos is not None
            and raised_pos[1] > rest_pos[1] + 0.020,
            details=f"rest={rest_pos}, posed={raised_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
